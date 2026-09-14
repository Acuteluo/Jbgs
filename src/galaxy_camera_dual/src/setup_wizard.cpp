// 文件: setup_wizard.cpp
// 作用: 交互式相机/传感器配置向导(camera_setup_wizard)。
//
// 面向"全新 Ubuntu 22.04 + 刚接好设备"的场景, 一个程序同时完成
// 环境检查 与 四模块接线确认, 结束后自动写好配置:
//   1. 环境检查: Galaxy SDK 初始化、枚举大恒 GigE 相机(序列号/IP/MAC)、
//      扫描 /dev/video*(红外候选)、扫描 /dev/ttyUSB*(传感器候选);
//      枚举不到设备时打印逐项排障清单(网卡网段/MTU/防火墙/SDK 安装)。
//   2. 左右大恒确认: 逐台全屏预览(按 ENTER 结束), 测试者目视后回答
//      这台是左相机还是右相机 => 序列号绑定到对应 JSON;
//   3. 红外确认: 逐个 /dev/video 候选预览, 确认热像画面正常 => 写入
//      infrared_camera.json 的 device_path;
//   4. 传感器确认: 对每个串口候选发 Modbus 读寄存器, 展示 ~6 秒实时
//      温湿度/CO2, 确认正常 => 写入 sensor_driver.yaml 的 device_name;
//   5. 双机同时取流测带宽, 自动写入两台 JSON 能稳定跑通的最高
//      frame_rate_hz(以及配套 throughput_limit_bps)。相机交换机若经
//      USB2(480Mbps)接入，双 5MP GigE 不允许“降帧勉强写入”，必须换
//      USB3+ 或把交换机直插板载网口。红外/传感器可继续走扩展坞 USB；
//      向导按实际载波、USB 拓扑最低速率和双机完整帧实测决定配置。
//   6. 全部通过后把 launch.json 的 sim_mode 置 false(真机模式),
//      所有被修改的文件先备份为 *.bak。
//
// 用法(在本机 source 过 install 环境的终端直接运行, 不依赖 ROS 运行时):
//   ros2 run galaxy_camera_dual camera_setup_wizard          # 交互式
//   ros2 run galaxy_camera_dual camera_setup_wizard --check  # 只检查不写入
//   ros2 run galaxy_camera_dual camera_setup_wizard --tune-fps  # 只测/写帧率

#include "galaxy_camera_dual/galaxy_device.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <DxImageProc.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <vector>

#include <unistd.h>

namespace fs = std::filesystem;
using galaxy_camera_dual::DeviceAddress;
using galaxy_camera_dual::DeviceInfo;
using galaxy_camera_dual::GalaxyDevice;

namespace
{

// ----------------------------------------------------------------------
// 终端小工具
// ----------------------------------------------------------------------

std::string trim(std::string s)
{
    while (!s.empty() && (s.back() == ' ' || s.back() == '\n' ||
                          s.back() == '\r' || s.back() == '\t'))
    {
        s.pop_back();
    }
    while (!s.empty() && s.front() == ' ')
    {
        s.erase(s.begin());
    }
    return s;
}

/// 提问并读取一行
std::string ask(const std::string & prompt)
{
    std::cout << prompt;
    std::cout.flush();
    std::string line;
    if (!std::getline(std::cin, line))
    {
        std::cout << "\n输入流结束, 退出向导" << std::endl;
        std::exit(2);
    }
    return trim(line);
}

bool ask_yes(const std::string & prompt, bool default_yes = true)
{
    const std::string a = ask(prompt + (default_yes ? " [Y/n]: " : " [y/N]: "));
    if (a.empty())
    {
        return default_yes;
    }
    return a == "y" || a == "Y" || a == "yes" || a == "YES";
}

/// 工程根目录: JBGS_ROOT 优先, 否则从当前目录向上找含 config/launch.json 的目录
std::string find_project_root()
{
    const char * env = getenv("JBGS_ROOT");
    if (env != nullptr &&
        fs::exists(fs::path(env) / "config" / "launch.json"))
    {
        return env;
    }
    fs::path cur = fs::current_path();
    for (int i = 0; i < 6; ++i)
    {
        if (fs::exists(cur / "config" / "launch.json"))
        {
            return cur.string();
        }
        if (!cur.has_parent_path() || cur.parent_path() == cur)
        {
            break;
        }
        cur = cur.parent_path();
    }
    return "";
}

/// 读文件全部文本
std::string read_text(const std::string & path)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        return "";
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/// 带备份的键值替换: 把 "key": "old" 的第一个出现换成 "key": "new"
/// (只做最小改动, 保留 JSON 注释与排版)
bool patch_json_string_value(
    const std::string & path, const std::string & key,
    const std::string & value)
{
    const std::string text = read_text(path);
    if (text.empty())
    {
        std::cout << "  [错误] 无法读取 " << path << std::endl;
        return false;
    }
    const std::string needle = "\"" + key + "\": \"";
    const size_t pos = text.find(needle);
    if (pos == std::string::npos)
    {
        std::cout << "  [错误] " << path << " 中找不到键 \"" << key << "\""
                  << std::endl;
        return false;
    }
    const size_t val_begin = pos + needle.size();
    const size_t val_end = text.find('"', val_begin);
    if (val_end == std::string::npos)
    {
        return false;
    }
    const std::string old_val = text.substr(val_begin, val_end - val_begin);
    if (old_val == value)
    {
        std::cout << "  [跳过] " << fs::path(path).filename().string()
                  << " 的 " << key << " 已经是 \"" << value << "\"" << std::endl;
        return true;
    }
    std::string patched = text;
    patched.replace(val_begin, val_end - val_begin, value);
    std::error_code ec;
    fs::copy_file(path, path + ".bak",
                  fs::copy_options::overwrite_existing, ec);
    std::ofstream f(path, std::ios::trunc);
    if (!f.is_open())
    {
        std::cout << "  [错误] 无法写入 " << path << std::endl;
        return false;
    }
    f << patched;
    std::cout << "  [已写入] " << fs::path(path).filename().string() << ": "
              << key << ": \"" << old_val << "\" -> \"" << value
              << "\" (原文件备份为 .bak)" << std::endl;
    return true;
}

/// 替换 JSON 数值(int/float, 不带引号)。保留原文件排版, 只改第一个匹配键。
bool patch_json_number_value(
    const std::string & path, const std::string & key,
    const std::string & value)
{
    const std::string text = read_text(path);
    if (text.empty())
    {
        std::cout << "  [错误] 无法读取 " << path << std::endl;
        return false;
    }
    const std::string needle = "\"" + key + "\":";
    const size_t pos = text.find(needle);
    if (pos == std::string::npos)
    {
        std::cout << "  [错误] " << path << " 中找不到键 \"" << key << "\""
                  << std::endl;
        return false;
    }
    size_t val_begin = pos + needle.size();
    while (val_begin < text.size() &&
           (text[val_begin] == ' ' || text[val_begin] == '\t'))
    {
        ++val_begin;
    }
    size_t val_end = val_begin;
    while (val_end < text.size() &&
           text[val_end] != ',' && text[val_end] != '\n' &&
           text[val_end] != '}' && text[val_end] != ' ')
    {
        ++val_end;
    }
    const std::string old_val = text.substr(val_begin, val_end - val_begin);
    if (old_val == value)
    {
        std::cout << "  [跳过] " << fs::path(path).filename().string()
                  << " 的 " << key << " 已经是 " << value << std::endl;
        return true;
    }
    std::string patched = text;
    patched.replace(val_begin, val_end - val_begin, value);
    std::error_code ec;
    fs::copy_file(path, path + ".bak",
                  fs::copy_options::overwrite_existing, ec);
    std::ofstream f(path, std::ios::trunc);
    if (!f.is_open())
    {
        std::cout << "  [错误] 无法写入 " << path << std::endl;
        return false;
    }
    f << patched;
    std::cout << "  [已写入] " << fs::path(path).filename().string() << ": "
              << key << ": " << old_val << " -> " << value
              << " (原文件备份为 .bak)" << std::endl;
    return true;
}

/// 从 JSON 读一个数值键; 找不到返回 default_v。
double read_json_number(
    const std::string & path, const std::string & key, double default_v)
{
    const std::string text = read_text(path);
    const std::string needle = "\"" + key + "\":";
    const size_t pos = text.find(needle);
    if (pos == std::string::npos)
    {
        return default_v;
    }
    size_t i = pos + needle.size();
    while (i < text.size() && (text[i] == ' ' || text[i] == '\t'))
    {
        ++i;
    }
    try
    {
        return std::stod(text.substr(i));
    }
    catch (...)
    {
        return default_v;
    }
}

/// 列出 /dev/video* 候选(排序)
std::vector<std::string> list_video_devices()
{
    std::vector<std::string> out;
    DIR * dir = opendir("/dev");
    if (dir == nullptr)
    {
        return out;
    }
    while (dirent * e = readdir(dir))
    {
        const std::string name = e->d_name;
        if (name.rfind("video", 0) == 0)
        {
            out.push_back("/dev/" + name);
        }
    }
    closedir(dir);
    std::sort(out.begin(), out.end());
    return out;
}

/// 列出串口候选(/dev/ttyUSB* 与 /dev/ttyACM*)
std::vector<std::string> list_serial_devices()
{
    std::vector<std::string> out;
    DIR * dir = opendir("/dev");
    if (dir == nullptr)
    {
        return out;
    }
    while (dirent * e = readdir(dir))
    {
        const std::string name = e->d_name;
        if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0)
        {
            out.push_back("/dev/" + name);
        }
    }
    closedir(dir);
    std::sort(out.begin(), out.end());
    return out;
}

// ----------------------------------------------------------------------
// 传感器: 最小 Modbus RTU 主站(只实现 0x03 读保持寄存器)
// ----------------------------------------------------------------------

uint16_t crc16_modbus(const std::vector<uint8_t> & data)
{
    uint16_t crc = 0xFFFF;
    for (const uint8_t b : data)
    {
        crc ^= b;
        for (int i = 0; i < 8; ++i)
        {
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                            : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;   // 低字节在前发送
}

class SerialPort
{
public:
    ~SerialPort() { close(); }

    bool open(const std::string & path, speed_t baud = B9600)
    {
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0)
        {
            return false;
        }
        fcntl(fd_, F_SETFL, 0);
        termios tty{};
        if (tcgetattr(fd_, &tty) != 0)
        {
            close();
            return false;
        }
        cfmakeraw(&tty);
        // cfmakeraw() 只设置原始 8N1 位格式，不保证打开接收器；部分
        // USB-RS485 适配器在 CREAD 未置位时会“能打开但永远收不到应答”。
        tty.c_cflag |= CLOCAL | CREAD;
        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;   // 0.5s 无数据即返回(总超时由上层控制)
        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            close();
            return false;
        }
        tcflush(fd_, TCIOFLUSH);
        return true;
    }

    void close()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    /// 读保持寄存器; 成功返回寄存器值数组
    bool read_holding(
        uint8_t addr, uint16_t reg, uint16_t count,
        std::vector<uint16_t> * values)
    {
        std::vector<uint8_t> frame = {addr, 0x03,
            static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF),
            static_cast<uint8_t>(count >> 8),
            static_cast<uint8_t>(count & 0xFF)};
        const uint16_t crc = crc16_modbus(frame);
        frame.push_back(static_cast<uint8_t>(crc & 0xFF));   // CRC 低字节
        frame.push_back(static_cast<uint8_t>(crc >> 8));     // CRC 高字节
        if (::write(fd_, frame.data(), frame.size()) !=
            static_cast<ssize_t>(frame.size()))
        {
            return false;
        }
        const size_t expect = 5 + 2 * count;   // addr+func+len+data+crc2
        std::vector<uint8_t> reply;
        reply.reserve(expect);
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(800);
        while (reply.size() < expect &&
               std::chrono::steady_clock::now() < deadline)
        {
            uint8_t buf[64];
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n > 0)
            {
                reply.insert(reply.end(), buf, buf + n);
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        if (reply.size() < expect || reply[0] != addr || reply[1] != 0x03 ||
            reply[2] != 2 * count)
        {
            return false;
        }
        std::vector<uint8_t> body(reply.begin(), reply.end() - 2);
        const uint16_t crc_recv = static_cast<uint16_t>(
            reply[reply.size() - 2] | (reply[reply.size() - 1] << 8));
        if (crc16_modbus(body) != crc_recv)
        {
            return false;
        }
        for (uint16_t i = 0; i < count; ++i)
        {
            values->push_back(static_cast<uint16_t>(
                (reply[3 + 2 * i] << 8) | reply[4 + 2 * i]));
        }
        return true;
    }

private:
    int fd_ = -1;
};

// ----------------------------------------------------------------------
// GigE 网段自动修复: SDK 以网卡"主地址"为源 IP, 相机不同段时
// 枚举正常但 open 超时(-14)。检测并(经确认后)自动修复。
// ----------------------------------------------------------------------

/// 列出主机 IPv4: (地址, 网卡名)。
/// `ip -o addr` 格式为 `8: enx... inet 169.254.10.10/16 ...`，
/// 接口名不带 `dev` 关键字；不要按普通 `ip addr` 的格式解析。
std::vector<std::pair<std::string, std::string>> list_host_ipv4()
{
    std::vector<std::pair<std::string, std::string>> out;
    if (auto * fp = popen("ip -4 -o addr show 2>/dev/null", "r"))
    {
        char line[512];
        while (fgets(line, sizeof(line), fp) != nullptr)
        {
            std::istringstream ss{std::string(line)};
            std::string index_colon;
            std::string dev;
            std::string family;
            std::string cidr;
            if (!(ss >> index_colon >> dev >> family >> cidr) ||
                family != "inet")
            {
                continue;
            }
            const size_t slash = cidr.find('/');
            if (slash == std::string::npos)
            {
                continue;
            }
            out.emplace_back(cidr.substr(0, slash), dev);
        }
        pclose(fp);
    }
    return out;
}

/// 相机网段(前两段, 如 169.254)
std::string cam_subnet(const std::string & ip)
{
    const size_t d1 = ip.find('.');
    const size_t d2 = (d1 == std::string::npos) ?
        std::string::npos : ip.find('.', d1 + 1);
    return (d2 == std::string::npos) ? ip : ip.substr(0, d2);
}

/// 配置千兆网卡以承受 5MP GigE 的 UDP 突发。rx_max 由调用方传入
/// (max_rx_ring 的结果; <=0 表示无法探测, 跳过 ring 调整)。
/// 调用者已经通过 sudo -v，nic 只能来自已校验的交互输入。MTU 和 RX
/// descriptor 均是易失设置，因此向导每次执行都会核验并可再次修复。
int current_rx_ring(const std::string & nic);

bool configure_gige_nic(const std::string & nic, int rx_max, int mtu = 9000)
{
    const std::string mtu_cmd =
        "sudo ip link set dev " + nic + " mtu " + std::to_string(mtu);
    if (std::system(mtu_cmd.c_str()) != 0)
    {
        std::cout << "  [错误] 无法把 " << nic << " 设为 MTU " << mtu
                  << std::endl;
        return false;
    }

    // RX ring 只调到驱动公布的上限。RTL8125B 等网卡上限仅 256, 硬设
    // 4096 必然 netlink error; RTL8153 默认常仅 100, 一张 5MP/8192B 包
    // 的图像约 615 个 UDP 包, 能调大就调大, 避免突发先于 NAPI 消费
    // 而在网卡内被丢包。已在上限时 r8169 对同值设置也会报错, 跳过。
    if (rx_max > 0 && current_rx_ring(nic) < rx_max)
    {
        const std::string ring_cmd =
            "sudo ethtool -G " + nic + " rx " + std::to_string(rx_max);
        if (std::system(ring_cmd.c_str()) != 0)
        {
            std::cout << "  [警告] " << nic << " 设置 RX ring="
                      << rx_max << " 失败" << std::endl;
        }
        else
        {
            std::cout << "  [已修复] " << nic << " 已设 MTU " << mtu
                      << "、RX ring " << rx_max << std::endl;
        }
    }
    else
    {
        std::cout << "  [已修复] " << nic << " 已设 MTU " << mtu
                  << (rx_max > 0 ? "(RX ring 已在驱动上限)" : "") << std::endl;
    }
    // GRO 会把 GVSP UDP 聚错, RTL8125/r8169 上残帧的常见原因; 每次都关。
    std::system(("sudo ethtool -K " + nic + " gro off 2>/dev/null").c_str());
    return true;
}

/// 将相机交换机网卡的易失配置做成 device-unit 绑定服务。扩展坞重插会
/// 重建 RTL8153 接口，普通 `ip addr add` / `ethtool -G` 随即失效；该
/// 服务在该网卡每次出现时恢复相机地址、巨帧与 RX ring。
/// rest_addrs: 同网卡上必须保留的其它地址(如 mid360 的 192.168.1.50/24)。
/// flush 只应发生在"专用于相机"的网卡上(无 NM 连接管理时才安装);
/// 与 mid360 等其它设备共用的网卡走 NM 配置持久化, 绝不装本服务。
bool install_gige_recovery_service(const std::string & nic,
                                   const std::string & cam_addr_cidr,
                                   const std::vector<std::string> & rest_addrs,
                                   int rx_max, int mtu = 9000)
{
    if (!ask_yes("  是否安装扩展坞重连后自动恢复 GigE 网络的系统服务?", true))
    {
        std::cout << "  [提示] 未安装持久恢复服务；扩展坞重插后需再次运行向导"
                  << std::endl;
        return true;
    }
    const std::string service = "jbgs-gige-" + nic + ".service";
    const fs::path tmp = fs::path("/tmp") / service;
    std::ofstream out(tmp);
    if (!out)
    {
        std::cout << "  [错误] 无法创建持久网络服务临时文件" << std::endl;
        return false;
    }
    out << "[Unit]\n"
        << "Description=JBGS GigE camera network recovery for " << nic << "\n"
        << "BindsTo=sys-subsystem-net-devices-" << nic << ".device\n"
        << "After=sys-subsystem-net-devices-" << nic << ".device\n\n"
        << "[Service]\nType=oneshot\n"
        << "ExecStart=/usr/sbin/ip link set dev " << nic << " mtu " << mtu << "\n"
        << "ExecStart=/usr/sbin/ip -4 addr flush dev " << nic << " scope global\n"
        << "ExecStart=/usr/sbin/ip addr add " << cam_addr_cidr
        << " dev " << nic << "\n";
    for (const auto & a : rest_addrs)
    {
        out << "ExecStart=/usr/sbin/ip addr add " << a << " dev " << nic << "\n";
    }
    if (rx_max > 0)
    {
        out << "ExecStart=/usr/sbin/ethtool -G " << nic
            << " rx " << rx_max << "\n";
    }
    out << "RemainAfterExit=yes\n\n"
        << "[Install]\nWantedBy=sys-subsystem-net-devices-" << nic << ".device\n";
    out.close();
    const std::string cmd =
        "sudo install -m 0644 " + tmp.string() + " /etc/systemd/system/" + service +
        " && sudo systemctl daemon-reload && sudo systemctl enable --now " + service;
    if (std::system(cmd.c_str()) != 0)
    {
        std::cout << "  [错误] 持久恢复服务安装失败" << std::endl;
        return false;
    }
    std::cout << "  [已修复] 已安装 " << service
              << "；扩展坞重连后将自动恢复 GigE 网络(保留同网卡其它地址)"
              << std::endl;
    return true;
}

/// 读取 `ethtool -g` 的 Current hardware settings/RX 值；不可读返回 -1。
int current_rx_ring(const std::string & nic)
{
    const std::string cmd = "ethtool -g " + nic + " 2>/dev/null";
    auto * fp = popen(cmd.c_str(), "r");
    if (fp == nullptr)
    {
        return -1;
    }
    bool current_section = false;
    int result = -1;
    char line[256];
    while (fgets(line, sizeof(line), fp) != nullptr)
    {
        const std::string value(line);
        if (value.find("Current hardware settings:") != std::string::npos)
        {
            current_section = true;
            continue;
        }
        if (current_section && value.rfind("RX:", 0) == 0)
        {
            std::istringstream ss(value.substr(3));
            ss >> result;
            break;
        }
    }
    pclose(fp);
    return result;
}

/// MAC 归一化: 只保留十六进制字符并转小写, 使 SDK 报告的
/// "84-47-09-.." 与 /sys 的 "84:47:09:.." 两种风格可互相比较。
std::string canonical_mac(const std::string & m)
{
    std::string out;
    for (const char c : m)
    {
        if (std::isxdigit(static_cast<unsigned char>(c)))
        {
            out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
    }
    return out;
}

/// 读 /sys/class/net/<nic>/address; 失败返回空串。
std::string nic_sys_mac(const std::string & nic)
{
    std::ifstream f("/sys/class/net/" + nic + "/address");
    std::string m;
    std::getline(f, m);
    return trim(m);
}

/// 检查相机网段 vs 主机网卡; 不同段时提供 sudo 自动修复。
/// 返回: true = 网段已就绪(原本就绪或修复成功)。
/// 枚举有线候选网卡(排除 lo/无线 wl*/常见虚拟口)。
std::vector<std::string> list_wired_nics()
{
    std::vector<std::string> cands;
    if (auto * fp = popen("ip -o link show 2>/dev/null", "r"))
    {
        char line[512];
        while (fgets(line, sizeof(line), fp) != nullptr)
        {
            std::string l(line);
            if (l.find(" lo:") != std::string::npos)
            {
                continue;
            }
            const size_t c1 = l.find(':');
            const size_t c2 = l.find(':', c1 + 1);
            if (c1 == std::string::npos || c2 == std::string::npos)
            {
                continue;
            }
            std::string name = l.substr(c1 + 2, c2 - c1 - 2);
            // 排除无线(wl*)与常见虚拟口
            if (name.rfind("wl", 0) == 0 ||
                name.rfind("tailscale", 0) == 0 ||
                name.rfind("Meta", 0) == 0 ||
                name.rfind("br-", 0) == 0 ||
                name.rfind("veth", 0) == 0 ||
                name.rfind("docker", 0) == 0)
            {
                continue;
            }
            cands.push_back(name);
        }
        pclose(fp);
    }
    return cands;
}

/// 选择连接相机交换机的网卡: 单候选自动采用, 多候选按编号选择。
/// 返回空串 = 用户放弃/无候选。
std::string select_wired_nic()
{
    const auto cands = list_wired_nics();
    if (cands.empty())
    {
        std::cout << "  [错误] 未发现任何有线网卡! 请插好扩展坞或 USB 网卡"
                     "(本机无板载网口)" << std::endl;
        return "";
    }
    if (cands.size() == 1)
    {
        std::cout << "  检测到唯一有线网卡: " << cands.front()
                  << " (自动采用)" << std::endl;
        return cands.front();
    }
    std::cout << "  检测到多块有线网卡:" << std::endl;
    for (size_t i = 0; i < cands.size(); ++i)
    {
        std::cout << "    [" << (i + 1) << "] " << cands[i] << std::endl;
    }
    std::cout << "  输入连接相机交换机的网卡编号: " << std::flush;
    std::string a;
    std::getline(std::cin, a);
    const int idx = std::atoi(a.c_str());
    if (idx >= 1 && idx <= static_cast<int>(cands.size()))
    {
        std::cout << "  已选择: " << cands[static_cast<size_t>(idx - 1)]
                  << std::endl;
        return cands[static_cast<size_t>(idx - 1)];
    }
    std::cout << "  [错误] 编号无效" << std::endl;
    return "";
}


/// 网口是否插线(有载波); 读取失败按无载波处理。
bool nic_has_carrier(const std::string & nic)
{
    std::ifstream f("/sys/class/net/" + nic + "/carrier");
    int c = 0;
    if (!(f >> c))
    {
        return false;
    }
    return c == 1;
}

/// 按 MAC 把 SDK 报告的"相机所在网卡"解析成网卡名; 找不到返回空。
/// require_carrier=true 时跳过无载波口: 交换机从扩展坞网口改插板载后,
/// SDK 仍可能报坞网口 MAC(坞以太网 DOWN, 但残留 169.254 地址)。
/// 扩展坞 USB(红外/传感器)不受影响, 这里只看以太网口。
std::string nic_name_from_mac(const std::string & mac,
                              bool require_carrier = true)
{
    if (mac.empty())
    {
        return "";
    }
    const std::string want = canonical_mac(mac);
    for (const std::string & name : list_wired_nics())
    {
        if (canonical_mac(nic_sys_mac(name)) != want)
        {
            continue;
        }
        if (require_carrier && !nic_has_carrier(name))
        {
            continue;
        }
        return name;
    }
    return "";
}

/// 相机交换机实际所在网口: 优先 SDK MAC 且有载波; 否则在同网段、有
/// 载波的有线口里选。skipped_down_nic 带回 SDK 误报的无载波口(若有)。
std::string resolve_camera_nic(const std::vector<DeviceInfo> & cams,
                               std::string * skipped_down_nic = nullptr)
{
    std::string down_from_mac;
    for (const auto & d : cams)
    {
        const std::string up = nic_name_from_mac(d.nic_mac, true);
        if (!up.empty())
        {
            return up;
        }
        const std::string any = nic_name_from_mac(d.nic_mac, false);
        if (!any.empty() && !nic_has_carrier(any))
        {
            down_from_mac = any;
        }
    }
    if (skipped_down_nic != nullptr)
    {
        *skipped_down_nic = down_from_mac;
    }
    const auto nics = list_host_ipv4();
    for (const auto & d : cams)
    {
        if (d.ip.empty())
        {
            continue;
        }
        for (const auto & n : nics)
        {
            if (cam_subnet(d.ip) == cam_subnet(n.first) &&
                nic_has_carrier(n.second))
            {
                return n.second;
            }
        }
    }
    return "";
}

/// 无载波有线口上若还挂着相机网段地址, 会抢走 169.254 路由(metric 优于
/// 板载口)。扩展坞改只挂 USB 设备后, 坞以太网常留下这种残留; 清 IP
/// 不影响坞上的红外/传感器 USB。
void cleanup_stale_addrs_on_down_nics(const std::string & live_nic,
                                      const std::string & cam_ip)
{
    const std::string subnet = cam_subnet(cam_ip);
    for (const std::string & nic : list_wired_nics())
    {
        if (nic == live_nic || nic_has_carrier(nic))
        {
            continue;
        }
        bool stale = false;
        for (const auto & p : list_host_ipv4())
        {
            if (p.second == nic && cam_subnet(p.first) == subnet)
            {
                stale = true;
                break;
            }
        }
        if (!stale)
        {
            continue;
        }
        std::cout << "  [清理] " << nic
                  << " 无载波但仍有相机网段地址(交换机已不在此口;"
                     " 坞上 USB 红外/传感器不受影响), 去掉残留 IP"
                  << std::endl;
        if (std::system(("sudo ip -4 addr flush dev " + nic).c_str()) != 0)
        {
            std::cout << "  [警告] 未能清理 " << nic << " 残留地址"
                      << std::endl;
        }
    }
}

/// 读 /sys/class/net/<nic>/mtu; 失败返回 -1。
int nic_mtu(const std::string & nic)
{
    std::ifstream f("/sys/class/net/" + nic + "/mtu");
    int v = -1;
    f >> v;
    return v;
}

/// 沿 sysfs 设备链找到 USB 各级链路的最低速率(Mbps)。不能只读最靠近
/// 设备的 speed：USB3 网卡若经 USB2 扩展坞上联，该值可能是 5000，
/// 但真正瓶颈是父级的 480。非 USB 设备返回 0。
int usb_path_min_speed_mbps(const fs::path & sysfs_device)
{
    std::error_code ec;
    fs::path p = fs::canonical(sysfs_device, ec);
    if (ec)
    {
        return 0;
    }
    int minimum = 0;
    for (int i = 0; i < 8 && p != p.root_path(); ++i)
    {
        std::ifstream vendor(p / "idVendor");
        std::ifstream speedf(p / "speed");
        if (vendor && speedf)
        {
            double sp = 0;
            if ((speedf >> sp) && sp > 0.0)
            {
                const int mbps = static_cast<int>(std::lround(sp));
                minimum = minimum == 0 ? mbps : std::min(minimum, mbps);
            }
        }
        p = p.parent_path();
    }
    return minimum;
}

/// 读相机网卡所在 USB 路径的实际瓶颈；板载/PCI 网卡返回 0。
int nic_usb_speed_mbps(const std::string & nic)
{
    return usb_path_min_speed_mbps("/sys/class/net/" + nic + "/device");
}

/// /dev/video* 与 /dev/ttyUSB*/ttyACM* 也可能挂在扩展坞或板载 USB。
/// 仅作拓扑报告：红外与 Modbus 本身带宽很小，不会据此拒绝设备。
int devnode_usb_speed_mbps(const std::string & devnode)
{
    const std::string name = fs::path(devnode).filename().string();
    if (name.rfind("video", 0) == 0)
    {
        return usb_path_min_speed_mbps(
            "/sys/class/video4linux/" + name + "/device");
    }
    if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0)
    {
        return usb_path_min_speed_mbps("/sys/class/tty/" + name + "/device");
    }
    return 0;
}

/// USB2 巨帧经常整帧残缺, 用标准 MTU; USB3/板载千兆才上 9000。
int recommended_mtu(int usb_mbps)
{
    if (usb_mbps > 0 && usb_mbps <= 480)
    {
        return 1500;
    }
    return 9000;
}

int64_t recommended_packet_size(int usb_mbps)
{
    if (usb_mbps > 0 && usb_mbps <= 480)
    {
        return 1500;
    }
    return 8192;
}

/// 以太网协商速率(Mb/s)。USB 网卡常显示 1000, 那只是 PHY, 不是 USB 总线。
int nic_eth_speed_mbps(const std::string & nic)
{
    std::ifstream f("/sys/class/net/" + nic + "/speed");
    int v = -1;
    f >> v;
    return v;
}

std::string nic_driver_name(const std::string & nic)
{
    std::error_code ec;
    const fs::path p = fs::canonical(
        "/sys/class/net/" + nic + "/device/driver", ec);
    if (ec)
    {
        return "";
    }
    return p.filename().string();
}

/// 给人看的链路类型: USB2 坞 / USB3 坞 / 板载直插网口。
std::string link_kind_label(int usb_mbps)
{
    if (usb_mbps > 0 && usb_mbps <= 480)
    {
        return "USB 2.0 扩展坞/USB 网卡";
    }
    if (usb_mbps >= 20000)
    {
        return "USB4/雷电 扩展坞网卡";
    }
    if (usb_mbps >= 10000)
    {
        return "USB 3.1/3.2 扩展坞网卡";
    }
    if (usb_mbps >= 5000)
    {
        return "USB 3.0 扩展坞网卡";
    }
    if (usb_mbps > 0)
    {
        return "USB 网卡";
    }
    return "板载有线网口(交换机直插)";
}

std::string aux_usb_path_label(int usb_mbps)
{
    if (usb_mbps <= 0)
    {
        return "非 USB / 无法从 sysfs 识别";
    }
    if (usb_mbps <= 480)
    {
        return "USB 2.0 (480 Mbps；对红外/传感器通常足够)";
    }
    return "USB 3+ (" + std::to_string(usb_mbps) + " Mbps)";
}

/// 红外、传感器不参与 GigE 带宽拒绝，但把它们实际 USB 上联打印出来，
/// 让“全接扩展坞”和“网线板载直插、USB 设备仍在坞上”两种拓扑一眼可见。
void print_aux_usb_topology(const char * title,
                            const std::vector<std::string> & devices)
{
    if (!devices.empty())
    {
        std::cout << "  " << title << " USB 拓扑:" << std::endl;
    }
    for (const auto & dev : devices)
    {
        const int speed = devnode_usb_speed_mbps(dev);
        std::cout << "  候选: " << dev << "  ["
                  << aux_usb_path_label(speed) << "]" << std::endl;
    }
}

void print_link_identity(const std::string & nic)
{
    const int usb = nic_usb_speed_mbps(nic);
    const int eth = nic_eth_speed_mbps(nic);
    const std::string drv = nic_driver_name(nic);
    const int mtu = recommended_mtu(usb);
    const int64_t pkt = recommended_packet_size(usb);
    std::cout << "  -------- 相机链路识别 --------" << std::endl;
    std::cout << "  网卡: " << nic << std::endl;
    std::cout << "  类型: " << link_kind_label(usb) << std::endl;
    if (usb > 0)
    {
        std::cout << "  USB 总线: " << usb << " Mbps"
                  << (usb <= 480 ? "  ← 这是真正瓶颈" : "  (USB3+, 瓶颈在千兆以太网)")
                  << std::endl;
    }
    else
    {
        std::cout << "  USB 总线: 无(PCI/板载网口)" << std::endl;
    }
    if (eth > 0)
    {
        std::cout << "  以太网协商: " << eth << " Mb/s";
        if (usb > 0 && usb <= 480 && eth >= 1000)
        {
            std::cout << " (PHY 千兆, 但 USB2 过不了这么多)";
        }
        std::cout << std::endl;
    }
    if (!drv.empty())
    {
        std::cout << "  驱动: " << drv << std::endl;
    }
    std::cout << "  据此预选: MTU " << mtu << ", GVSP 包长 " << pkt
              << " B, 双机帧率上限待实测" << std::endl;
    std::cout << "  ------------------------------" << std::endl;
}

/// 解析 `ethtool -g` 的 Pre-set maximums 段 RX 上限; 不可读返回 -1。
/// RTL8125B 等网卡上限仅 256: 硬设 4096 必然 netlink error。只有读到
/// 上限才能区分"当前值可再调大"与"已是硬件极限, 只能如实提示"。
int max_rx_ring(const std::string & nic)
{
    const std::string cmd = "ethtool -g " + nic + " 2>/dev/null";
    auto * fp = popen(cmd.c_str(), "r");
    if (fp == nullptr)
    {
        return -1;
    }
    bool max_section = false;
    int result = -1;
    char line[256];
    while (fgets(line, sizeof(line), fp) != nullptr)
    {
        const std::string value(line);
        if (value.find("Pre-set maximums:") != std::string::npos)
        {
            max_section = true;
            continue;
        }
        if (value.find("Current hardware settings:") != std::string::npos)
        {
            break;
        }
        if (max_section && value.rfind("RX:", 0) == 0)
        {
            std::istringstream ss(value.substr(3));
            ss >> result;
            break;
        }
    }
    pclose(fp);
    return result;
}

/// 生成一个落在相机网段内的主机侧地址(LLA 网段固定用 169.254.100.1/16,
/// 其它网段用相机 IP 前三段 + .1/24)。同一网卡允许同时挂多个网段地址
/// (实测 GigE 相机收流不受影响), 因此绝不需要为相机删除别的网段地址。
std::string host_addr_in_cam_subnet(const std::string & cam_ip)
{
    if (cam_ip.rfind("169.254.", 0) == 0)
    {
        return "169.254.100.1/16";
    }
    const size_t d = cam_ip.rfind('.');
    return (d == std::string::npos) ? "169.254.100.1/16" :
        cam_ip.substr(0, d) + ".1/24";
}

/// 解析 `ip -4 -o addr` 一行。NM 地址是 `inet CIDR brd x.x.x.x scope ...`,
/// `ip addr add` 的则是 `inet CIDR scope ...`——不能假定 scope 紧跟 CIDR。
bool parse_inet_addr_line(const std::string & line,
                          std::string * cidr, std::string * scope)
{
    std::istringstream ss(line);
    std::string index_colon, device, family, addr, tok;
    if (!(ss >> index_colon >> device >> family >> addr) || family != "inet")
    {
        return false;
    }
    std::string sc;
    while (ss >> tok)
    {
        if (tok == "scope")
        {
            ss >> sc;
            break;
        }
    }
    if (sc.empty())
    {
        return false;
    }
    if (cidr != nullptr)
    {
        *cidr = addr;
    }
    if (scope != nullptr)
    {
        *scope = sc;
    }
    return true;
}

/// 网卡上全部全局 IPv4(按内核添加顺序); 无则空列表。
std::vector<std::string> nic_global_ipv4s(const std::string & nic)
{
    std::vector<std::string> out;
    if (auto * fp = popen(
            ("ip -4 -o addr show dev " + nic + " 2>/dev/null").c_str(), "r"))
    {
        char line[512];
        while (fgets(line, sizeof(line), fp) != nullptr)
        {
            std::string addr, scope;
            if (parse_inet_addr_line(line, &addr, &scope) &&
                scope.rfind("global", 0) == 0)
            {
                out.push_back(addr);
            }
        }
        pclose(fp);
    }
    return out;
}

/// 网卡上首个可作 GigE 控制源的 IPv4。相机出厂的 169.254/16 就是
/// link-local；实机双机取流已验证它可作为 GxGVTL 源地址，不能误判为
/// “没有地址”。其它不可路由 scope 不参与判断。
std::string first_gige_control_ipv4(const std::string & nic)
{
    const std::string cmd = "ip -4 -o addr show dev " + nic + " 2>/dev/null";
    auto * fp = popen(cmd.c_str(), "r");
    if (fp == nullptr)
    {
        return "";
    }
    char line[512];
    std::string cidr;
    while (fgets(line, sizeof(line), fp) != nullptr)
    {
        std::string addr, scope;
        if (!parse_inet_addr_line(line, &addr, &scope))
        {
            continue;
        }
        if (scope.rfind("global", 0) == 0 || scope.rfind("link", 0) == 0)
        {
            cidr = addr;
            break;
        }
    }
    pclose(fp);
    return cidr;
}

/// 把网卡可持久化的全局 IPv4 重排为 cam_addr_cidr 在首位, 其余地址按原顺序跟后
/// (保留的原地址通过 kept_out 带出, 供 NM 持久化写回同一顺序)。
/// GxGVTL 实测以网卡"首地址"为控制通道源地址: 相机网段地址不在首位时,
/// 即使同网卡已配同段地址, open 仍超时(-14)——已在本机(6.8 内核)复现。
/// 重排会让该网卡上其它设备(如 mid360)闪断约 1 秒, 调用方须先征得同意。
bool reorder_nic_addresses_first(const std::string & nic,
                                 const std::string & cam_addr_cidr,
                                 std::vector<std::string> * kept_out = nullptr)
{
    std::vector<std::string> keep;
    if (auto * fp = popen(
            ("ip -4 -o addr show dev " + nic + " 2>/dev/null").c_str(), "r"))
    {
        char line[512];
        while (fgets(line, sizeof(line), fp) != nullptr)
        {
            std::string addr, scope;
            if (parse_inet_addr_line(line, &addr, &scope) &&
                scope.rfind("global", 0) == 0 && addr != cam_addr_cidr)
            {
                keep.push_back(addr);
            }
        }
        pclose(fp);
    }
    // 169.254/16 按内核规则是 scope link；这是合法的 GigE 相机控制源。
    // 不强行写 scope global（iproute2/内核会拒绝该组合）。
    std::string cmd = "sudo ip addr del " + cam_addr_cidr + " dev " + nic +
        " >/dev/null 2>&1; sudo ip -4 addr flush dev " + nic + " scope global";
    cmd += " && sudo ip addr add " + cam_addr_cidr + " dev " + nic;
    for (const auto & a : keep)
    {
        cmd += " && sudo ip addr add " + a + " dev " + nic;
    }
    const bool ok = std::system(cmd.c_str()) == 0;
    if (ok && kept_out != nullptr)
    {
        *kept_out = keep;
    }
    return ok;
}

/// 若 NetworkManager 正管理该网卡, 返回其 active 连接名; 否则返回空。
std::string nm_active_connection(const std::string & nic)
{
    // 字段名是 NAME 不是 CONNECTION(后者 nmcli 直接报错); NAME 在前时
    // 输出形如 "有线连接 2:enp45s0"。
    auto * fp = popen(
        "nmcli -t -f NAME,DEVICE con show --active 2>/dev/null", "r");
    if (fp == nullptr)
    {
        return "";
    }
    char line[512];
    std::string con;
    while (fgets(line, sizeof(line), fp) != nullptr)
    {
        const std::string l = trim(line);
        const size_t sep = l.rfind(':');
        if (sep == std::string::npos || l.substr(sep + 1) != nic)
        {
            continue;
        }
        con = l.substr(0, sep);
        break;
    }
    pclose(fp);
    return con;
}

/// 把重排后的地址序列写进 NM 连接配置, 保证重启后仍以相机地址为首位。
/// 返回 false = NM 未管理(调用方应考虑装恢复服务)或写配置失败。
bool persist_addresses_nm(const std::string & nic,
                          const std::string & cam_addr_cidr,
                          const std::vector<std::string> & rest_addrs,
                          int mtu = 9000)
{
    const std::string con = nm_active_connection(nic);
    if (con.empty())
    {
        return false;
    }
    std::string csv = cam_addr_cidr;
    for (const auto & a : rest_addrs)
    {
        csv += "," + a;
    }
    // 幂等: 配置已一致时静默跳过, 每次运行向导都不会重复扰动。
    // 注意 -g 对多字段是逐行输出: 第一行 method, 第二行 addresses。
    if (auto * fp = popen(("nmcli -g ipv4.method,ipv4.addresses con show '" +
                           con + "' 2>/dev/null").c_str(), "r"))
    {
        char line[256];
        std::string method, addrs;
        if (fgets(line, sizeof(line), fp) != nullptr)
        {
            method = trim(line);
        }
        if (fgets(line, sizeof(line), fp) != nullptr)
        {
            addrs = trim(line);
        }
        pclose(fp);
        if (method == "manual" && addrs == csv)
        {
            return true;
        }
    }
    // method 一并切 manual: DHCP 地址的添加顺序不受控, 无法保证相机
    // 网段地址恒在首位; 专用相机网卡本就不该靠 DHCP。
    const std::string mod = "sudo nmcli con mod '" + con +
        "' ipv4.method manual ipv4.addresses \"" + csv +
        "\" 802-3-ethernet.mtu " + std::to_string(mtu) +
        " connection.autoconnect yes";
    const std::string reapply =
        "sudo nmcli device reapply " + nic + " >/dev/null 2>&1";
    if (std::system(mod.c_str()) != 0 || std::system(reapply.c_str()) != 0)
    {
        std::cout << "  [警告] 未能写入 NM 连接配置(" << con
                  << ")；重启后需重新运行向导修复" << std::endl;
        return true;   // 实时修复已生效, 不算失败
    }
    std::cout << "  [已持久化] NM 连接 \"" << con
              << "\" 地址顺序已固定(相机网段在前), 重启后自动生效"
              << std::endl;
    return true;
}

/// NM 会重排地址；用 dispatcher 在该网口 up/reapply 后恢复相机网段
/// 地址在前。169.254/16 保持内核要求的 scope link，不伪造为 global。
bool install_gige_nm_dispatcher(const std::string & nic,
                                const std::string & cam_addr_cidr,
                                const std::vector<std::string> & rest_addrs)
{
    const std::string name = "99-jbgs-gige-" + nic;
    const fs::path tmp = fs::path("/tmp") / name;
    std::ofstream out(tmp);
    if (!out)
    {
        std::cout << "  [警告] 无法写 NM dispatcher 临时文件" << std::endl;
        return false;
    }
    out << "#!/bin/bash\n"
        << "# JBGS: 相机网段地址必须是该网口首个可用 IPv4 (GxGVTL)\n"
        << "IFACE=\"$1\"\nACTION=\"$2\"\n"
        << "[ \"$IFACE\" = \"" << nic << "\" ] || exit 0\n"
        << "case \"$ACTION\" in up|reapply|dhcp4-change) ;; *) exit 0 ;; esac\n"
        << "ip addr del " << cam_addr_cidr << " dev \"$IFACE\" >/dev/null 2>&1 || true\n"
        << "ip -4 addr flush dev \"$IFACE\" scope global\n"
        << "ip addr add " << cam_addr_cidr << " dev \"$IFACE\"\n";
    for (const auto & a : rest_addrs)
    {
        out << "ip addr add " << a << " dev \"$IFACE\"\n";
    }
    out << "ethtool -K \"$IFACE\" gro off >/dev/null 2>&1 || true\n";
    out.close();
    const std::string dest = "/etc/NetworkManager/dispatcher.d/" + name;
    const std::string cmd = "sudo install -m 0755 " + tmp.string() + " " + dest;
    if (std::system(cmd.c_str()) != 0)
    {
        std::cout << "  [警告] 未能安装 " << dest
                  << "；本次会话地址已修好, 重启后可能需再跑向导" << std::endl;
        return false;
    }
    std::cout << "  [已持久化] NM dispatcher " << name
              << " (重启后仍把相机地址恢复为首地址, 保留 mid360)"
              << std::endl;
    return true;
}

bool ensure_gige_subnet(const std::vector<DeviceInfo> & cams,
                        bool check_only, int * failures,
                        bool auto_fix = false)
{
    if (cams.empty())
    {
        return false;
    }

    // ---- 1. 相机所在网口: SDK MAC 优先, 但必须有载波 ----
    // 交换机从扩展坞网口改插板载后, SDK 仍可能报坞网口 MAC(坞以太网
    // DOWN, 残留 169.254)。红外/传感器继续走坞 USB, 只排除坞以太网。
    std::string skipped_down;
    std::string camera_nic = resolve_camera_nic(cams, &skipped_down);
    if (!skipped_down.empty() && skipped_down != camera_nic)
    {
        std::cout << "  [提示] SDK 仍报无载波网口 " << skipped_down
                  << "(多半是扩展坞网口还挂着旧地址); 相机交换机在 "
                  << (camera_nic.empty() ? "未知" : camera_nic)
                  << "。坞上 USB 红外/传感器不受影响" << std::endl;
    }
    if (camera_nic.empty())
    {
        std::cout << "  [警告] 无法确定相机所在网口, 跳过网卡诊断与修复"
                  << std::endl;
        ++(*failures);
        return false;
    }

    // ---- 2. 物理链路: 无载波时直说人话, 别让用户去猜 -14 ----
    if (!nic_has_carrier(camera_nic))
    {
        std::cout << "  [诊断] 相机所在网口 " << camera_nic
                  << " 无链路(NO-CARRIER): 网线未插或相机未供电,"
                     " 先查网线与 PoE" << std::endl;
        ++(*failures);
        return false;
    }

    const int usb_mbps = nic_usb_speed_mbps(camera_nic);
    const int want_mtu = recommended_mtu(usb_mbps);
    print_link_identity(camera_nic);

    const std::string cam_addr = host_addr_in_cam_subnet(cams.front().ip);
    if (!check_only)
    {
        cleanup_stale_addrs_on_down_nics(camera_nic, cams.front().ip);
        // GRO 会把 GVSP UDP 聚错; 即使 MTU/ring 已就绪也要关。
        std::system(("sudo ethtool -K " + camera_nic +
                     " gro off 2>/dev/null").c_str());
    }

    // ---- 3. 控制通道就绪: 相机网段地址必须是该网卡首个可用 IPv4 ----
    // GxGVTL 以网卡首地址为控制源(本机实测: 同网卡挂着 192.168.1.50 在
    // 首位时两台相机 open 全部超时, 相机地址调到首位后立即恢复)。
    // 169.254 的 scope link 是内核标准行为，实机可正常枚举与取流。
    auto cam_is_first_control_address = [&]()
    {
        const std::string now = first_gige_control_ipv4(camera_nic);
        return !now.empty() &&
            cam_subnet(now) == cam_subnet(cams.front().ip);
    };
    const std::string primary = first_gige_control_ipv4(camera_nic);
    if (!cam_is_first_control_address())
    {
        if (primary.empty())
        {
            std::cout << "  [诊断] " << camera_nic << " 没有可用 IPv4;"
                         " 相机在 " << cam_subnet(cams.front().ip)
                      << ".x 段, open 会超时(-14)" << std::endl;
        }
        else
        {
            std::cout << "  [诊断] " << camera_nic << " 首个全局地址 "
                      << primary << " 不在相机网段("
                      << cam_subnet(cams.front().ip)
                      << ".x); GxGVTL 以网卡首个全局 IPv4 为控制源, "
                         "open 会超时(-14)" << std::endl;
        }
        if (check_only)
        {
            ++(*failures);
            return false;
        }
        if (!primary.empty())
        {
            std::cout << "  [说明] 修复会把 " << cam_addr << " 调到该网卡"
                         "首地址, 其余地址(如 mid360 的 " << primary
                      << ")原序跟后; 期间同网卡设备闪断约 1 秒" << std::endl;
        }
        if (!auto_fix && !ask_yes("  是否自动修复(需要 sudo 密码)?", true))
        {
            std::cout << "  [提示] 手动修复命令:" << std::endl;
            std::cout << "      sudo ip addr add " << cam_addr << " dev "
                      << camera_nic
                      << "  # 必须排在该网卡 IPv4 地址列表首位" << std::endl;
            ++(*failures);
            return false;
        }
        if (auto_fix)
        {
            std::cout << "  [自动修复] --tune-fps 直接调整地址顺序"
                      << std::endl;
        }
        if (std::system("sudo -v") != 0)
        {
            std::cout << "  [错误] sudo 授权失败, 未修改网络配置" << std::endl;
            ++(*failures);
            return false;
        }
        std::vector<std::string> kept;
        if (!reorder_nic_addresses_first(camera_nic, cam_addr, &kept))
        {
            std::cout << "  [错误] 地址重排失败" << std::endl;
            ++(*failures);
            return false;
        }
        std::cout << "  [已修复] " << cam_addr << " 已是 " << camera_nic
                  << " 首个可用地址" << std::endl;
        persist_addresses_nm(camera_nic, cam_addr, kept, want_mtu);
        // NM reapply 可能重排地址，再强制一次地址顺序。
        if (!cam_is_first_control_address())
        {
            std::cout << "  [提示] NM 重排了相机网段地址, 再恢复首地址"
                      << std::endl;
            if (!reorder_nic_addresses_first(camera_nic, cam_addr, nullptr))
            {
                std::cout << "  [错误] 修复后首地址仍不匹配, 放弃"
                          << std::endl;
                ++(*failures);
                return false;
            }
        }
        if (!cam_is_first_control_address())
        {
            std::cout << "  [错误] 修复后首地址仍不匹配, 放弃" << std::endl;
            ++(*failures);
            return false;
        }
        if (!nm_active_connection(camera_nic).empty() &&
            cam_addr.rfind("169.254.", 0) == 0)
        {
            install_gige_nm_dispatcher(camera_nic, cam_addr, kept);
        }
    }
    else
    {
        std::cout << "  [网段] 已就绪: " << camera_nic << " 首个可用地址 "
                  << primary << " 覆盖相机网段" << std::endl;
        if (!check_only)
        {
            std::vector<std::string> rest;
            for (const auto & a : nic_global_ipv4s(camera_nic))
            {
                if (a != cam_addr)
                {
                    rest.push_back(a);
                }
            }
            persist_addresses_nm(camera_nic, cam_addr, rest, want_mtu);
            if (!cam_is_first_control_address())
            {
                reorder_nic_addresses_first(camera_nic, cam_addr, nullptr);
            }
            if (!nm_active_connection(camera_nic).empty() &&
                cam_addr.rfind("169.254.", 0) == 0)
            {
                install_gige_nm_dispatcher(camera_nic, cam_addr, rest);
            }
        }
    }

    // ---- 4. 突发容量: MTU 按链路选择; RX ring 调到驱动上限 ----
    const int mtu_now = nic_mtu(camera_nic);
    const int ring_now = current_rx_ring(camera_nic);
    const int ring_max = max_rx_ring(camera_nic);
    bool need_tune = false;
    if (mtu_now >= 0 && mtu_now != want_mtu)
    {
        std::cout << "  [诊断] " << camera_nic << " MTU=" << mtu_now
                  << ", 本链路建议 " << want_mtu
                  << (want_mtu >= 9000 ? "(5MP 巨帧 8192B 包)" : "(USB2 不用巨帧)")
                  << std::endl;
        need_tune = true;
    }
    if (ring_now >= 0 && ring_max > ring_now)
    {
        std::cout << "  [诊断] " << camera_nic << " RX ring=" << ring_now
                  << ", 驱动上限 " << ring_max << "; 建议调到上限" << std::endl;
        need_tune = true;
    }
    else if (ring_now >= 0 && ring_max > 0 && ring_now >= ring_max &&
             ring_now < 512)
    {
        std::cout << "  [提示] " << camera_nic << " RX ring=" << ring_now
                  << " 已是驱动上限(" << ring_max << "), 无法再调大; "
                     "以取流验收的残帧计数为准"
                  << std::endl;
    }
    if (need_tune)
    {
        if (check_only)
        {
            ++(*failures);
            return false;
        }
        if ((!auto_fix &&
             !ask_yes("  是否自动设置 MTU " + std::to_string(want_mtu) +
                      "、RX ring " +
                      std::to_string(ring_max > 0 ? ring_max : 4096) + "?",
                      true)) ||
            std::system("sudo -v") != 0 ||
            !configure_gige_nic(camera_nic, ring_max, want_mtu))
        {
            ++(*failures);
            return false;
        }
    }

    // ---- 5. 持久化分工 ----
    // NM 管理: 地址顺序/MTU 已写入连接, 169.254 另用 dispatcher 强制全局
    // 首位。USB 扩展坞重插仍需 MTU/RX ring 恢复服务(交互模式才问)。
    if (!check_only && !auto_fix)
    {
        std::vector<std::string> rest_for_service;
        for (const auto & a : nic_global_ipv4s(camera_nic))
        {
            if (a != cam_addr)
            {
                rest_for_service.push_back(a);
            }
        }
        const fs::path service_path = fs::path("/etc/systemd/system") /
            ("jbgs-gige-" + camera_nic + ".service");
        if (nm_active_connection(camera_nic).empty() &&
            !fs::exists(service_path))
        {
            if (std::system("sudo -v") != 0 ||
                !install_gige_recovery_service(camera_nic, cam_addr,
                                               rest_for_service, ring_max,
                                               want_mtu))
            {
                ++(*failures);
                return false;
            }
        }
        else if (fs::exists(service_path))
        {
            // 已有服务: 若内容不含其余地址, 提示会冲掉 mid360。不自动删,
            // 因为扩展坞路径正靠它恢复 MTU/ring; 交互里可重装覆盖。
            const std::string body = read_text(service_path.string());
            bool missing_rest = false;
            for (const auto & a : rest_for_service)
            {
                if (body.find(a) == std::string::npos)
                {
                    missing_rest = true;
                    break;
                }
            }
            if (missing_rest && !rest_for_service.empty())
            {
                std::cout << "  [警告] jbgs-gige-" << camera_nic
                          << ".service 未包含同网卡其它地址(如 mid360), "
                             "重插坞会冲掉它们。将按当前地址重写该服务"
                          << std::endl;
                if (std::system("sudo -v") == 0)
                {
                    install_gige_recovery_service(
                        camera_nic, cam_addr, rest_for_service, ring_max,
                        want_mtu);
                }
            }
        }
    }
    return true;
}

// ----------------------------------------------------------------------
// 双机同时取流测带宽 / 自动选最高稳定帧率
// ----------------------------------------------------------------------

/// MER-500-14GC Bayer8 一帧约 5,038,848 字节。
constexpr double kMer500PayloadBytes = 2592.0 * 1944.0;

int64_t throughput_for_fps(double fps, int ncam, int usb_mbps)
{
    int64_t need = static_cast<int64_t>(
        std::llround(fps * kMer500PayloadBytes * 1.15));
    int64_t cap = 55000000;
    if (usb_mbps > 0 && usb_mbps <= 480)
    {
        // USB2 实用吞吐约 28-32MB/s, 再给红外/键鼠留一点。
        cap = 28000000 / std::max(1, ncam);
    }
    if (need <= 0)
    {
        return cap;
    }
    return std::max<int64_t>(8000000, std::min(need, cap));
}

/// 按真机节点同款参数配流: 包长 + 限速/包间隔 + 可选软触发。
/// packet_size: USB2 用 1500, USB3/板载千兆用 8192。
bool apply_stream_settings(
    GalaxyDevice & dev, double fps, int64_t throughput_bps,
    int64_t packet_size)
{
    if (packet_size <= 0)
    {
        packet_size = 8192;
    }
    dev.setInt(GX_DEV_INT_COMMAND_TIMEOUT, 1000);
    dev.setAcquisitionBufferNumber(5);
    dev.setEnum(GX_DS_ENUM_RESEND_MODE, GX_DS_RESEND_MODE_ON);
    if (!dev.setInt(GX_INT_GEV_PACKETSIZE, packet_size))
    {
        int64_t actual = 0;
        if (dev.getInt(GX_INT_GEV_PACKETSIZE, &actual))
        {
            std::cout << "  [提示] 未能设置 packet_size=" << packet_size
                      << ", 相机协商值 " << actual << std::endl;
        }
    }
    bool throughput_ok = false;
    if (throughput_bps > 0 &&
        dev.isImplemented(GX_INT_DEVICE_LINK_THROUGHPUT_LIMIT) &&
        dev.isImplemented(GX_ENUM_DEVICE_LINK_THROUGHPUT_LIMIT_MODE))
    {
        throughput_ok =
            dev.setEnum(GX_ENUM_DEVICE_LINK_THROUGHPUT_LIMIT_MODE,
                        GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ON) &&
            dev.setInt(GX_INT_DEVICE_LINK_THROUGHPUT_LIMIT, throughput_bps);
    }
    if (!throughput_ok)
    {
        int64_t tick = 0;
        int64_t delay = 6000;
        const double bytes_per_sec = std::max(
            8.0e6, static_cast<double>(
                       throughput_bps > 0 ? throughput_bps : 18000000));
        if (dev.getInt(GX_INT_TIMESTAMP_TICK_FREQUENCY, &tick) && tick > 0)
        {
            delay = std::max<int64_t>(1, static_cast<int64_t>(std::llround(
                (static_cast<double>(packet_size) / bytes_per_sec) *
                static_cast<double>(tick))));
        }
        dev.setInt(GX_INT_GEV_PACKETDELAY, delay);
    }
    if (fps > 0.0)
    {
        if (!dev.setEnum(GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON) ||
            !dev.setEnum(GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE))
        {
            std::cout << "  [错误] 无法切换到软触发, 帧率不受控" << std::endl;
            return false;
        }
    }
    else
    {
        dev.setEnum(GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    }
    return true;
}

struct DualProbeResult
{
    // ready 只表示两台设备确实完成了打开、参数设置和开始采集。打开失败
    // (例如别的程序占用相机、刚重插网卡尚未稳定)绝不能被误判成带宽不足，
    // 更不能据此把已经验证过的帧率降下来。
    bool ready = false;
    bool ok = false;
    size_t complete[2] = {0, 0};
    size_t incomplete[2] = {0, 0};
};

/// 双机同时软触发取流 seconds 秒。生产节点就是这样跑的, 单机连拍不能代表带宽。
DualProbeResult probe_dual_stream(
    const std::string & sn0, const std::string & sn1,
    double fps, int64_t throughput_bps, int seconds, int64_t packet_size)
{
    DualProbeResult out;
    GalaxyDevice devs[2];
    const std::string sns[2] = {sn0, sn1};
    std::vector<uint8_t> buffers[2];
    GX_FRAME_DATA frames[2]{};
    for (int i = 0; i < 2; ++i)
    {
        DeviceAddress addr;
        addr.serial_number = sns[i];
        std::string err;
        if (!devs[i].open(addr, &err))
        {
            std::cout << "  [失败] SN " << sns[i] << " 无法打开: " << err
                      << std::endl;
            if (i == 1)
            {
                devs[0].close();
            }
            return out;
        }
        if (!apply_stream_settings(devs[i], fps, throughput_bps, packet_size))
        {
            devs[0].close();
            if (i == 1)
            {
                devs[1].close();
            }
            return out;
        }
        int64_t payload = 0;
        if (!devs[i].getInt(GX_INT_PAYLOAD_SIZE, &payload) || payload <= 0)
        {
            payload = static_cast<int64_t>(kMer500PayloadBytes);
        }
        buffers[i].assign(static_cast<size_t>(payload), 0);
        frames[i].pImgBuf = buffers[i].data();
        if (!devs[i].startAcquisition())
        {
            std::cout << "  [失败] SN " << sns[i] << " 无法开始采集" << std::endl;
            devs[0].close();
            if (i == 1)
            {
                devs[1].close();
            }
            return out;
        }
    }
    out.ready = true;

    std::atomic<bool> running{true};
    std::atomic<size_t> complete0{0};
    std::atomic<size_t> complete1{0};
    std::atomic<size_t> incomplete0{0};
    std::atomic<size_t> incomplete1{0};
    std::thread grabbers[2];
    grabbers[0] = std::thread([&]() {
        while (running.load())
        {
            const GX_STATUS st = devs[0].grab(&frames[0], 400);
            if (st == GX_STATUS_SUCCESS &&
                frames[0].nStatus == GX_FRAME_STATUS_SUCCESS)
            {
                complete0.fetch_add(1);
            }
            else if (st == GX_STATUS_SUCCESS)
            {
                incomplete0.fetch_add(1);
            }
        }
    });
    grabbers[1] = std::thread([&]() {
        while (running.load())
        {
            const GX_STATUS st = devs[1].grab(&frames[1], 400);
            if (st == GX_STATUS_SUCCESS &&
                frames[1].nStatus == GX_FRAME_STATUS_SUCCESS)
            {
                complete1.fetch_add(1);
            }
            else if (st == GX_STATUS_SUCCESS)
            {
                incomplete1.fetch_add(1);
            }
        }
    });
    std::thread triggers[2];
    const auto period = std::chrono::microseconds(
        static_cast<int64_t>(1000000.0 / fps));
    for (int i = 0; i < 2; ++i)
    {
        triggers[i] = std::thread([&, i]() {
            auto next = std::chrono::steady_clock::now() + period * i / 2;
            std::this_thread::sleep_until(next);
            while (running.load())
            {
                devs[i].sendCommand(GX_COMMAND_TRIGGER_SOFTWARE);
                next += period;
                std::this_thread::sleep_until(next);
            }
        });
    }
    // 两台相机刚切换软触发时，首批 GVSP 包会受到设备端配置生效、USB
    // 网卡 RX 队列建链的影响。它们不代表稳态带宽；先预热再清零计数，
    // 否则 --check 会把随后 35/35 完整帧的 12fps 误报为不稳定。
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    complete0.store(0);
    complete1.store(0);
    incomplete0.store(0);
    incomplete1.store(0);
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    running.store(false);
    for (int i = 0; i < 2; ++i)
    {
        devs[i].stopAcquisition();
    }
    for (int i = 0; i < 2; ++i)
    {
        if (triggers[i].joinable())
        {
            triggers[i].join();
        }
        if (grabbers[i].joinable())
        {
            grabbers[i].join();
        }
        devs[i].close();
    }
    out.complete[0] = complete0.load();
    out.complete[1] = complete1.load();
    out.incomplete[0] = incomplete0.load();
    out.incomplete[1] = incomplete1.load();
    const double expect = fps * static_cast<double>(seconds);
    const double min_ok = std::max(3.0, expect * 0.75);
    out.ok = true;
    for (int i = 0; i < 2; ++i)
    {
        const double got = static_cast<double>(out.complete[i]);
        const double bad = static_cast<double>(out.incomplete[i]);
        const double tot = got + bad;
        const bool enough = got >= min_ok;
        const bool clean = (bad < 0.5) || (tot > 0.0 && bad / tot <= 0.05);
        if (!enough || !clean)
        {
            out.ok = false;
        }
    }
    return out;
}

void print_dual_probe(const DualProbeResult & r, double fps)
{
    if (!r.ready)
    {
        std::cout << "  [双机 " << fps
                  << " fps] 未进入采集状态(不是带宽结论)" << std::endl;
        return;
    }
    std::cout << "  [双机 " << fps << " fps] 完整帧="
              << r.complete[0] << "/" << r.complete[1]
              << " 残帧=" << r.incomplete[0] << "/" << r.incomplete[1]
              << (r.ok ? "  通过" : "  不合格") << std::endl;
}

struct StreamTune
{
    double fps = 0.0;
    int64_t throughput_bps = 0;
    int64_t packet_size = 8192;
    int usb_mbps = 0;
    int eth_mbps = -1;
    int mtu = 1500;
    int rx_ring = -1;
    std::string nic;
};

void print_stream_settings_summary(const StreamTune & tune)
{
    std::cout << "  -------- 实测后的相机上限 --------" << std::endl;
    std::cout << "  链路: " << (tune.nic.empty() ? "?" : tune.nic)
              << "  (" << link_kind_label(tune.usb_mbps) << ")" << std::endl;
    if (tune.eth_mbps > 0)
    {
        std::cout << "  以太网协商: " << tune.eth_mbps << " Mb/s" << std::endl;
    }
    if (tune.fps > 0.0)
    {
        std::cout << "  双机稳定帧率: " << tune.fps << " fps  (写入 JSON)"
                  << std::endl;
        std::cout << "  每台吞吐上限: " << (tune.throughput_bps / 1000000)
                  << " MB/s" << std::endl;
        std::cout << "  GVSP 包长: " << tune.packet_size << " B" << std::endl;
        std::cout << "  网卡 MTU: " << tune.mtu
                  << "  RX ring: " << tune.rx_ring << std::endl;
    }
    else
    {
        std::cout << "  双机稳定帧率: 未达到  (不写入, 请换更快链路)"
                  << std::endl;
        std::cout << "  本链路预选包长: " << tune.packet_size
                  << " B, MTU " << tune.mtu << std::endl;
    }
    std::cout << "  ----------------------------------" << std::endl;
}

void print_link_upgrade_advice(const std::string & camera_nic, int usb_mbps)
{
    if (usb_mbps <= 0 || usb_mbps > 480)
    {
        return;
    }
    std::cout << "  [换口] 当前 " << camera_nic << " 挂在 USB 2.0(" << usb_mbps
              << " Mbps)上。两台 MER-500-14GC 满分辨率即使降到 2fps"
                 " 也经常整帧残缺, 不要勉强写入残帧配置。" << std::endl;
    std::cout << "  请任选一种更快链路后重新运行本向导:" << std::endl;
    for (const auto & nic : list_wired_nics())
    {
        if (nic == camera_nic)
        {
            continue;
        }
        const int usb = nic_usb_speed_mbps(nic);
        const bool up = nic_has_carrier(nic);
        if (usb == 0)
        {
            std::cout << "    - 板载网口 " << nic
                      << (up ? " (已插线)" : " (当前未插线)")
                      << " — 把交换机输出线直插小电脑网口" << std::endl;
        }
        else if (usb >= 5000)
        {
            std::cout << "    - USB3 网卡 " << nic << " (" << usb
                      << " Mbps)" << std::endl;
        }
    }
    std::cout << "    - 把扩展坞改插到机身 USB3/USB4 口(蓝色口或标 SS 的口;"
                 " lsusb -t 应显示 5000M/10000M/20000M, 而不是 480M)"
              << std::endl;
}

/// 单机连续采集探活: 判断 GVSP 在这条链路上能不能拿到完整帧。
bool probe_one_camera(const std::string & sn, int64_t packet_size,
                      int seconds, size_t * complete, size_t * incomplete)
{
    GalaxyDevice dev;
    DeviceAddress addr;
    addr.serial_number = sn;
    std::string err;
    if (!dev.open(addr, &err))
    {
        std::cout << "  [失败] SN " << sn << " 无法打开: " << err << std::endl;
        return false;
    }
    if (!apply_stream_settings(dev, 0.0, 0, packet_size))
    {
        dev.close();
        return false;
    }
    int64_t payload = 0;
    if (!dev.getInt(GX_INT_PAYLOAD_SIZE, &payload) || payload <= 0)
    {
        payload = static_cast<int64_t>(kMer500PayloadBytes);
    }
    std::vector<uint8_t> buffer(static_cast<size_t>(payload));
    GX_FRAME_DATA frame{};
    frame.pImgBuf = buffer.data();
    if (!dev.startAcquisition())
    {
        dev.close();
        return false;
    }
    size_t ok = 0;
    size_t bad = 0;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(seconds);
    while (std::chrono::steady_clock::now() < deadline)
    {
        const GX_STATUS st = dev.grab(&frame, 400);
        if (st == GX_STATUS_SUCCESS &&
            frame.nStatus == GX_FRAME_STATUS_SUCCESS)
        {
            ++ok;
        }
        else if (st == GX_STATUS_SUCCESS)
        {
            ++bad;
        }
    }
    dev.stopAcquisition();
    dev.close();
    if (complete != nullptr)
    {
        *complete = ok;
    }
    if (incomplete != nullptr)
    {
        *incomplete = bad;
    }
    return ok >= 3 && bad == 0;
}

/// 从高到低试双机同时取流, 返回最高稳定帧率。失败 fps=0, 不勉强写入。
StreamTune find_max_stable_fps(
    const std::vector<DeviceInfo> & cams, const std::string & camera_nic)
{
    StreamTune tune;
    tune.nic = camera_nic;
    if (cams.size() < 2)
    {
        return tune;
    }
    tune.usb_mbps = camera_nic.empty() ? 0 : nic_usb_speed_mbps(camera_nic);
    tune.eth_mbps = camera_nic.empty() ? -1 : nic_eth_speed_mbps(camera_nic);
    tune.packet_size = recommended_packet_size(tune.usb_mbps);
    tune.mtu = recommended_mtu(tune.usb_mbps);
    tune.rx_ring = camera_nic.empty() ? -1 : current_rx_ring(camera_nic);
    const bool usb2 = tune.usb_mbps > 0 && tune.usb_mbps <= 480;

    // 双 5MP GigE 经 USB2 无论怎样限速都属于勉强运行：与红外、键鼠等
    // 同坞设备共享总线时更容易残帧。此处硬拒绝，不探测低帧率、更不写 JSON。
    if (usb2)
    {
        std::cout << "  [拒绝配置] 相机交换机网卡实际经 USB2 "
                  << "(" << tune.usb_mbps << " Mbps) 上联；双 5MP GigE "
                     "不允许写入降帧配置。" << std::endl;
        print_link_upgrade_advice(camera_nic, tune.usb_mbps);
        return tune;
    }
    // 交换机到主机若只协商到百兆，同样不能给双 5MP 写一个“看似能跑”的值。
    if (tune.eth_mbps > 0 && tune.eth_mbps < 1000)
    {
        std::cout << "  [拒绝配置] " << camera_nic << " 仅协商到 "
                  << tune.eth_mbps << " Mb/s；请检查网线/交换机端口，"
                     "恢复千兆后再测。" << std::endl;
        return tune;
    }

    if (!GalaxyDevice::initLibrary())
    {
        std::cout << "  [错误] Galaxy SDK 初始化失败, 无法测帧率" << std::endl;
        return tune;
    }

    auto probe_first_cam = [&](int64_t psz) -> bool
    {
        size_t one_ok = 0;
        size_t one_bad = 0;
        std::cout << "  单机探活 SN " << cams[0].serial_number
                  << " (连续采集, packet=" << psz << ")..." << std::endl;
        const bool live = probe_one_camera(
            cams[0].serial_number, psz, 3, &one_ok, &one_bad);
        std::cout << "  [单机] 完整帧=" << one_ok << " 残帧=" << one_bad
                  << (live ? "  通过" : "  不合格") << std::endl;
        return live;
    };

    bool one_live = probe_first_cam(tune.packet_size);
    if (!one_live && tune.packet_size > 1500)
    {
        std::cout << "  [提示] 巨帧 " << tune.packet_size
                  << " 拿不到完整帧, 交换机/中间链路多半不支持 jumbo,"
                     " 改试 GVSP 1500 (主机 MTU 可仍为 9000)" << std::endl;
        if (probe_first_cam(1500))
        {
            tune.packet_size = 1500;
            one_live = true;
        }
    }
    if (!one_live)
    {
        std::cout << "  [提示] 单机自由采集不合格(RTL8125 RX ring=256 时常见),"
                     " 仍按生产路径做双机软触发+限速探测" << std::endl;
    }

    std::vector<int> cands;
    if (tune.eth_mbps >= 2500)
    {
        // 2.5G 上联可让两台各自千兆的相机接近额定 14fps；是否真的可行
        // 仍由下面的完整帧实测决定。
        cands = {14, 12, 10, 8, 6, 4};
    }
    else
    {
        // RTL8125 r8169 RX ring 上限 256: 12fps 双机实测时有时刚过线、
        // 复测又不够帧, 最高从 10 起测更稳。
        if (tune.rx_ring > 0 && tune.rx_ring <= 256)
        {
            cands = {10, 8, 6, 4};
        }
        else
        {
            cands = {12, 10, 8, 6, 4};
        }
    }
    bool camera_access_failed = false;
    auto try_dual_cands = [&]() -> bool
    {
        for (const int fps_i : cands)
        {
            const double fps = static_cast<double>(fps_i);
            const int64_t tput = throughput_for_fps(fps, 2, tune.usb_mbps);
            std::cout << "  探测双机 " << fps_i << " fps (packet="
                      << tune.packet_size << ", 每台限速 "
                      << (tput / 1000000) << " MB/s)..." << std::endl;
            DualProbeResult r = probe_dual_stream(
                cams[0].serial_number, cams[1].serial_number, fps, tput, 3,
                tune.packet_size);
            print_dual_probe(r, fps);
            if (!r.ready)
            {
                camera_access_failed = true;
                std::cout << "  [中止] 相机未能进入采集状态；保留现有 JSON "
                             "配置，不把访问/占用故障误降为低帧率。"
                          << std::endl;
                return false;
            }
            // 首轮刚恢复采集时可能有少量触发周期尚未稳定。只在“确实已
            // 进入采集但恰好未过线”时复测一次；两轮都失败才下调帧率。
            if (!r.ok)
            {
                std::cout << "  [复测] " << fps_i
                          << " fps 首轮未过线，重新验证一次..." << std::endl;
                r = probe_dual_stream(
                    cams[0].serial_number, cams[1].serial_number, fps, tput,
                    3, tune.packet_size);
                print_dual_probe(r, fps);
                if (!r.ready)
                {
                    camera_access_failed = true;
                    std::cout << "  [中止] 复测时相机未进入采集状态；保留"
                                 "现有 JSON 配置。"
                              << std::endl;
                    return false;
                }
            }
            if (r.ok)
            {
                tune.fps = fps;
                tune.throughput_bps = tput;
                std::cout << "  [帧率] 本链路最高稳定 " << fps_i
                          << " fps, 将写入两台相机 JSON" << std::endl;
                return true;
            }
        }
        return false;
    };
    bool dual_ok = try_dual_cands();
    if (!dual_ok && !camera_access_failed && tune.packet_size > 1500)
    {
        std::cout << "  [提示] 巨帧双机不稳定, 改试 GVSP 1500" << std::endl;
        tune.packet_size = 1500;
        dual_ok = try_dual_cands();
    }
    GalaxyDevice::closeLibrary();
    if (dual_ok)
    {
        return tune;
    }
    if (usb2)
    {
        std::cout << "  [结论] 单机能出图, 但双机在 USB2 上拿不到稳定完整帧。"
                     "助手不会写入一个勉强的残帧帧率。" << std::endl;
        print_link_upgrade_advice(camera_nic, tune.usb_mbps);
    }
    else
    {
        std::cout << "  [失败] 双机同时取流在候选帧率下都不稳定。"
                     "请检查网线/交换机/PoE 后重跑向导" << std::endl;
    }
    return tune;
}

std::string camera_nic_of(const std::vector<DeviceInfo> & cams)
{
    return resolve_camera_nic(cams);
}

bool write_stream_tune(const std::string & root, const StreamTune & tune)
{
    if (tune.fps <= 0.0)
    {
        return false;
    }
    bool ok = true;
    char fps_txt[32];
    std::snprintf(fps_txt, sizeof(fps_txt), "%.1f", tune.fps);
    const std::string bps = std::to_string(tune.throughput_bps);
    const std::string psz = std::to_string(tune.packet_size);
    for (const char * file : {"/config/galaxy_camera_1.json",
                              "/config/galaxy_camera_2.json"})
    {
        ok = patch_json_number_value(root + file, "frame_rate_hz", fps_txt) &&
             ok;
        ok = patch_json_number_value(
                 root + file, "throughput_limit_bps", bps) &&
             ok;
        ok = patch_json_number_value(root + file, "packet_size", psz) && ok;
    }
    return ok;
}

// ----------------------------------------------------------------------
// 预览
// ----------------------------------------------------------------------

/// 大恒预览: 打开指定序列号相机, 全屏显示直到按 ENTER。
/// fps>0 时用与真机节点相同的软触发, 避免 USB2 上自由采集 14fps 打满总线。
bool preview_daheng(const std::string & sn, double fps, int64_t throughput_bps,
                    int64_t packet_size)
{
    // 库生命周期配对: 第 1 步枚举后已 closeLibrary(), 预览前必须重新
    // init(GalaxyDevice::open 不会自动初始化库, 否则 GXUpdateAllDeviceList
    // 直接失败)。
    if (!GalaxyDevice::initLibrary())
    {
        std::cout << "  [错误] Galaxy SDK 初始化失败, 无法预览" << std::endl;
        return false;
    }
    GalaxyDevice dev;
    DeviceAddress addr;
    addr.serial_number = sn;
    std::string err;
    if (!dev.open(addr, &err))
    {
        std::cout << "  [错误] 打开相机 " << sn << " 失败: " << err << std::endl;
        std::cout << "  [提示] 最常见原因是相机与网卡不同网段(能枚举到但"
                     "收不了流): 把网卡配到相机同段"
                     "(sudo ip addr add 169.254.x.x/16 dev <网卡>)"
                     "或用 GalaxyIPConfig 改相机 IP" << std::endl;
        GalaxyDevice::closeLibrary();
        return false;
    }
    const double use_fps = fps > 0.0 ? fps : 4.0;
    const int64_t use_tput = throughput_bps > 0 ? throughput_bps :
        throughput_for_fps(use_fps, 2, 0);
    const int64_t use_pkt = packet_size > 0 ? packet_size : 8192;
    if (!apply_stream_settings(dev, use_fps, use_tput, use_pkt))
    {
        std::cout << "  [错误] 无法配置取流参数，预览不能保证出帧" << std::endl;
        dev.close();
        GalaxyDevice::closeLibrary();
        return false;
    }
    int64_t payload = 0;
    if (!dev.getInt(GX_INT_PAYLOAD_SIZE, &payload) || payload <= 0)
    {
        payload = 2592 * 1944;   // MER-500-14GC 标称分辨率兜底
    }
    std::vector<uint8_t> buffer(static_cast<size_t>(payload));
    GX_FRAME_DATA frame{};
    frame.pImgBuf = buffer.data();
    if (!dev.startAcquisition())
    {
        std::cout << "  [错误] 相机 " << sn << " 无法开始采集" << std::endl;
        dev.close();
        GalaxyDevice::closeLibrary();
        return false;
    }
    std::atomic<bool> trig_run{true};
    std::thread trig([&]() {
        const auto period = std::chrono::microseconds(
            static_cast<int64_t>(1000000.0 / use_fps));
        auto next = std::chrono::steady_clock::now();
        while (trig_run.load())
        {
            dev.sendCommand(GX_COMMAND_TRIGGER_SOFTWARE);
            next += period;
            std::this_thread::sleep_until(next);
        }
    });
    const std::string win =
        "preview - SN " + sn + "  (press ENTER to finish)";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::setWindowProperty(win, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    std::cout << "  预览中: SN " << sn << "。目视判断画面朝向, "
              << "然后在预览窗口按 ENTER 关闭(窗口拿不到焦点时, "
              << "在终端直接回车等效; 终端输入 s 回车=跳过)..." << std::endl;
    int ok_frames = 0;
    auto fps_t0 = std::chrono::steady_clock::now();
    bool accepted = false;
    while (true)
    {
        cv::Mat bgr;
        const GX_STATUS st = dev.grab(&frame, 500);
        if (st == GX_STATUS_SUCCESS && frame.nStatus == GX_FRAME_STATUS_SUCCESS)
        {
            ++ok_frames;
            DX_PIXEL_COLOR_FILTER bayer = BAYERGR;
            bool bayer_ok = false;
            switch (frame.nPixelFormat)
            {
            case GX_PIXEL_FORMAT_BAYER_GR8:
                bayer = BAYERGR; bayer_ok = true; break;
            case GX_PIXEL_FORMAT_BAYER_RG8:
                bayer = BAYERRG; bayer_ok = true; break;
            case GX_PIXEL_FORMAT_BAYER_GB8:
                bayer = BAYERGB; bayer_ok = true; break;
            case GX_PIXEL_FORMAT_BAYER_BG8:
                bayer = BAYERBG; bayer_ok = true; break;
            default: break;
            }
            if (bayer_ok)
            {
                cv::Mat rgb(frame.nHeight, frame.nWidth, CV_8UC3);
                if (DxRaw8toRGB24(
                        frame.pImgBuf, rgb.data,
                        static_cast<VxUint32>(frame.nWidth),
                        static_cast<VxUint32>(frame.nHeight),
                        RAW2RGB_NEIGHBOUR, bayer, false) == DX_OK)
                {
                    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
                }
            }
        }
        const double fps_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - fps_t0).count();
        char fps_txt[64];
        std::snprintf(fps_txt, sizeof(fps_txt), "%.1f fps",
            fps_elapsed > 0.5 ? ok_frames / fps_elapsed : 0.0);
        if (bgr.empty())
        {
            bgr = cv::Mat(960, 1280, CV_8UC3, cv::Scalar(30, 30, 30));
            cv::putText(bgr, "NO FRAME from SN " + sn, cv::Point(40, 480),
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2);
        }
        const std::string head = "SN " + sn + "  " + fps_txt +
            (ok_frames >= 5 ? "  |  press ENTER to bind"
                            : "  |  no/low frames - ENTER disabled (ESC=skip)");
        cv::putText(bgr, head, cv::Point(30, 70),
                    cv::FONT_HERSHEY_SIMPLEX, 1.1,
                    ok_frames >= 5 ? cv::Scalar(0, 255, 255)
                                   : cv::Scalar(0, 0, 255), 2);
        cv::imshow(win, bgr);
        int key = cv::waitKey(30);
        // 终端回车兜底: Wayland/SSH 场景预览窗口常拿不到键盘焦点。
        // 终端有输入时: 空行(回车)=等效窗口 ENTER, s+回车=等效 ESC,
        // 其余单字符忽略; 逐行消费, 不影响之后的终端问答顺序。
        {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(STDIN_FILENO, &rfds);
            timeval tv{0, 0};
            if (select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv) > 0)
            {
                std::string line;
                std::getline(std::cin, line);
                if (!line.empty() && (line[0] == 's' || line[0] == 'S'))
                {
                    key = 27;
                }
                else if (line.empty() || line == "\r")
                {
                    key = 13;
                }
            }
        }
        // 帧数不足 5 帧时 ENTER 无效: 防止把无流相机绑进配置。
        // ESC = 明确放弃该相机(跳过绑定)。
        if (key == 27)
        {
            std::cout << "  [跳过] SN " << sn << " 预览异常(帧数 "
                      << ok_frames << ")已跳过绑定" << std::endl;
            break;
        }
        if (key == 13)
        {
            if (ok_frames >= 5)
            {
                accepted = true;
                break;
            }
            std::cout << "  [拦截] 该相机 " << fps_txt
                      << " 帧率异常, ENTER 已禁用; 检查链路后按 ESC 跳过"
                      << std::endl;
        }
    }
    cv::destroyWindow(win);
    trig_run.store(false);
    dev.stopAcquisition();
    if (trig.joinable())
    {
        trig.join();
    }
    dev.close();
    GalaxyDevice::closeLibrary();
    return accepted;
}

/// 红外(V4L2)预览: 返回 true = 用户确认画面是热像且正常
bool preview_ir(const std::string & path)
{
    cv::VideoCapture cap(path, cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        return false;
    }
    // 不强设分辨率/帧率: 热像芯原生 256x192, 硬设 640x512 会协商失败
    // 或得到拉伸花屏画面, 曾被误判为"不是红外"。交由 V4L2 按设备能力
    // 协商默认格式。
    const std::string win = "preview - IR " + path;
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::Mat frame;
    // 热像机芯上电后有数秒快门校准(FFC), 期间输出全黑帧; 等待窗口要
    // 覆盖这段, 否则黑屏画面同样会被误判为设备异常。
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (cap.read(frame) && !frame.empty())
        {
            cv::putText(frame, path + "  |  answer in terminal",
                        cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                        cv::Scalar(0, 255, 255), 2);
            cv::imshow(win, frame);
            cv::waitKey(1);
            break;
        }
    }
    if (frame.empty())
    {
        std::cout << "  [错误] " << path << " 3 秒内没有可用画面" << std::endl;
        cv::destroyWindow(win);
        cap.release();
        return false;
    }
    const std::string answer = ask(
        "  请查看预览窗口；热像且正常吗? [Y=确认 / n=下一个]: ");
    cv::destroyWindow(win);
    cap.release();
    return answer.empty() || answer == "Y" || answer == "y";
}

}  // namespace

// ======================================================================

int main(int argc, char ** argv)
{
    bool check_only = false;
    bool tune_only = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--check" || a == "-c")
        {
            check_only = true;
        }
        else if (a == "--tune-fps" || a == "--tune-stream")
        {
            tune_only = true;
        }
    }
    const std::string root = find_project_root();
    std::cout << "==============================================" << std::endl;
    std::cout << " 涵洞巡检 相机/传感器 配置向导" << std::endl;
    std::cout << "   工程根: " << (root.empty() ? "<未找到!>" : root) << std::endl;
    std::cout << "   模式  : "
              << (check_only ? "只检查(--check)"
                             : (tune_only ? "只测/写帧率(--tune-fps)"
                                          : "交互式配置"))
              << std::endl;
    std::cout << "==============================================" << std::endl;
    if (root.empty())
    {
        std::cout << "[错误] 未定位到工程根(config/launch.json), "
                     "请先 source install/setup.bash 再运行" << std::endl;
        return 1;
    }

    int failures = 0;

    // ============ 1. 大恒相机枚举 ============
    std::cout << "\n[1/4] 大恒 GigE 相机(Galaxy SDK)..." << std::endl;
    std::vector<DeviceInfo> cams;
    if (GalaxyDevice::initLibrary())
    {
        cams = GalaxyDevice::listDevices(1500);
        GalaxyDevice::closeLibrary();
    }
    else
    {
        std::cout << "  [错误] Galaxy SDK 初始化失败! 排查清单:" << std::endl;
        std::cout << "  - 大恒 Galaxy SDK 是否安装(是否运行过 SDK 的 install.sh)"
                  << std::endl;
        std::cout << "  - LD_LIBRARY_PATH 是否含 SDK 库目录(source install 后"
                     "由本工程自动带上)" << std::endl;
        std::cout << "  - .cti 传输层文件(GxGVTL.cti 等)是否在 libgxiapi.so "
                     "同目录(本工程已随包携带; 若用官方安装包则需先跑其 "
                     "install.sh)" << std::endl;
        ++failures;
    }
    for (const auto & d : cams)
    {
        std::cout << "  找到: [" << d.index << "] " << d.model_name
                  << " SN=" << d.serial_number << " IP=" << d.ip
                  << " MAC=" << d.mac;
        const std::string cam_nic = nic_name_from_mac(d.nic_mac, true);
        const std::string cam_nic_any = nic_name_from_mac(d.nic_mac, false);
        if (!cam_nic.empty())
        {
            std::cout << " (所在网卡 " << cam_nic << ")";
        }
        else if (!cam_nic_any.empty())
        {
            std::cout << " (SDK 报网卡 " << cam_nic_any
                      << ", 无载波已忽略)";
        }
        std::cout << std::endl;
    }
    // ---- 网段自动诊断与修复(必须在预览之前: 不同段时预览也打不开) ----
    // --tune-fps 也要修地址: 换口后不修则 open 超时, 测不出帧率。
    if (!ensure_gige_subnet(cams, check_only, &failures, tune_only))
    {
        std::cout << "  [警告] 网段未就绪, 相机预览/绑定将不可用" << std::endl;
    }

    if (check_only && cams.size() >= 2 && failures == 0)
    {
        const std::string nic = camera_nic_of(cams);
        const int usb = nic.empty() ? 0 : nic_usb_speed_mbps(nic);
        const double json_fps = read_json_number(
            root + "/config/galaxy_camera_1.json", "frame_rate_hz", 8.0);
        const int64_t json_tput = static_cast<int64_t>(read_json_number(
            root + "/config/galaxy_camera_1.json", "throughput_limit_bps",
            55000000.0));
        std::cout << "  双机同时取流验收(JSON " << json_fps
                  << " fps, 非单机连拍)..." << std::endl;
        if (!GalaxyDevice::initLibrary())
        {
            ++failures;
        }
        else
        {
            DualProbeResult r = probe_dual_stream(
                cams[0].serial_number, cams[1].serial_number,
                json_fps,
                json_tput > 0 ? json_tput : throughput_for_fps(json_fps, 2, usb),
                3, recommended_packet_size(usb));
            GalaxyDevice::closeLibrary();
            print_dual_probe(r, json_fps);
            if (r.ready && !r.ok)
            {
                std::cout << "  [复测] 当前 JSON 帧率首轮未过线，重新验证一次..."
                          << std::endl;
                if (GalaxyDevice::initLibrary())
                {
                    r = probe_dual_stream(
                        cams[0].serial_number, cams[1].serial_number,
                        json_fps,
                        json_tput > 0 ? json_tput :
                        throughput_for_fps(json_fps, 2, usb),
                        3, recommended_packet_size(usb));
                    GalaxyDevice::closeLibrary();
                    print_dual_probe(r, json_fps);
                }
                else
                {
                    r.ready = false;
                }
            }
            if (!r.ready)
            {
                ++failures;
                std::cout << "  [失败] 相机没有进入采集状态(可能被其他程序占用、"
                             "刚重插仍在枚举，或控制通道异常)。保留当前 JSON，"
                             "不会据此推荐降帧；排除占用后重跑 --check。"
                          << std::endl;
            }
            else if (!r.ok)
            {
                ++failures;
                std::cout << "  [失败] 当前配置帧率在本链路上不稳定。"
                             "下面自动探测最高可用帧率(不写入):"
                          << std::endl;
                const StreamTune rec = find_max_stable_fps(cams, nic);
                print_stream_settings_summary(rec);
                if (rec.fps > 0.0)
                {
                    std::cout << "  [建议] 运行向导或 --tune-fps 将自动写入 "
                              << rec.fps << " fps" << std::endl;
                }
            }
        }
    }

    // 新机自举: 网卡没有任何 IPv4(或全为错段)时枚举必为空 ——
    // 给网卡加 link-local 地址后重新枚举(相机出厂默认 169.254.x.x)。
    if (cams.size() < 2 && !check_only && !tune_only)
    {
        if (ask_yes("  是否给连接相机的网卡配置 169.254.100.1/16(首地址)"
                    " 后重新枚举?", false))
        {
            const std::string nic = select_wired_nic();
            if (nic.empty())
            {
                std::cout << "  [错误] 未选择网卡, 无法自举" << std::endl;
                ++failures;
                return 1;
            }
            if (std::system("sudo -v") != 0)
            {
                std::cout << "  [错误] sudo 授权失败，未执行网络修改" << std::endl;
                ++failures;
                return 1;
            }
            // 相机出厂默认 169.254.x.x; 用固定主机地址 169.254.100.1/16,
            // 且必须调到网卡首地址(GxGVTL 以首地址为控制源, 非首位会
            // open 超时)。其余既有地址(如 mid360)原序保留。
            std::vector<std::string> kept;
            if (!reorder_nic_addresses_first(nic, "169.254.100.1/16", &kept))
            {
                std::cout << "  [错误] 地址配置失败, 检查网卡名" << std::endl;
                ++failures;
                return 1;
            }
            std::cout << "  [已修复] 169.254.100.1/16 已加入 " << nic
                      << " (首地址)" << std::endl;
            persist_addresses_nm(nic, "169.254.100.1/16", kept,
                                 recommended_mtu(nic_usb_speed_mbps(nic)));
            configure_gige_nic(nic, max_rx_ring(nic),
                               recommended_mtu(nic_usb_speed_mbps(nic)));
            std::cout << "  [已修复] 重新枚举..." << std::endl;
            if (GalaxyDevice::initLibrary())
            {
                cams = GalaxyDevice::listDevices(1500);
                GalaxyDevice::closeLibrary();
            }
            for (const auto & d : cams)
            {
                std::cout << "  找到: [" << d.index << "] "
                          << d.model_name << " SN=" << d.serial_number
                          << " IP=" << d.ip;
                const std::string cam_nic2 = nic_name_from_mac(d.nic_mac, true);
                if (!cam_nic2.empty())
                {
                    std::cout << " (所在网卡 " << cam_nic2 << ")";
                }
                std::cout << std::endl;
            }
        }
    }
    if (cams.size() < 2)
    {
        std::cout << "  [警告] 只找到 " << cams.size()
                  << " 台大恒相机(需要 2 台)。排障清单:" << std::endl;
        std::cout << "  - 网线/交换机/供电(PoE)是否接好" << std::endl;
        std::cout << "  - 相机默认 IP(169.254.x.x)与电脑网卡是否同网段:"
                     " 网卡需配同段地址或开 DHCP" << std::endl;
        std::cout << "  - 巨帧: 建议 sudo ip link set <网卡> mtu 9000"
                  << std::endl;
        std::cout << "  - 防火墙: GigE 走 UDP 收流, 可临时 sudo ufw disable 验证"
                  << std::endl;
        std::cout << "  - 可用大恒 GalaxyIPConfig 工具把相机 IP 改到固定网段"
                  << std::endl;
        failures += (2 - static_cast<int>(cams.size()));
    }

    StreamTune tuned;
    if (cams.size() >= 2 && !check_only)
    {
        std::cout << "\n[1b] 双机同时取流测带宽, 自动选择最高稳定帧率..."
                  << std::endl;
        tuned = find_max_stable_fps(cams, camera_nic_of(cams));
        print_stream_settings_summary(tuned);
        if (tuned.fps <= 0.0)
        {
            ++failures;
            std::cout << "  [警告] 未找到稳定双机帧率, 不写入勉强配置"
                      << std::endl;
        }
        if (tune_only)
        {
            if (tuned.fps > 0.0)
            {
                std::cout << "\n[写配置] 只写入帧率/限速/包长" << std::endl;
                if (!write_stream_tune(root, tuned))
                {
                    ++failures;
                }
            }
            else
            {
                std::cout << "\n[写配置] 跳过: 当前链路不够双机稳定取流,"
                             " 请换 USB3 口或直插板载网口后再跑向导"
                          << std::endl;
            }
            std::cout << "\n=============================================="
                      << std::endl;
            std::cout << (failures == 0 ? " 检查全部通过 ✓" :
                          " 存在未通过项, 见上方")
                      << std::endl;
            std::cout << "=============================================="
                      << std::endl;
            return failures == 0 ? 0 : 1;
        }
    }

    // ============ 2. 左右相机确认与绑定 ============
    std::string left_sn;
    std::string right_sn;
    if (!check_only && !tune_only && cams.size() >= 2)
    {
        if (tuned.fps <= 0.0)
        {
            std::cout << "\n[2/4] 跳过左右预览绑定: 当前链路无法稳定双机取流,"
                         " 请按上方建议换口后再跑向导" << std::endl;
        }
        else
        {
        std::cout << "\n[2/4] 左右相机确认(逐台全屏预览, 看画面回答是哪侧)..."
                  << std::endl;
        for (const auto & d : cams)
        {
            if (!preview_daheng(d.serial_number, tuned.fps,
                                tuned.throughput_bps, tuned.packet_size))
            {
                std::cout << "  [跳过] SN " << d.serial_number
                          << " 未通过预览，不参与左右绑定" << std::endl;
                continue;
            }
            while (true)
            {
                const std::string a = ask(
                    "  刚才预览的相机(SN " + d.serial_number +
                    ") 是哪一侧? [L=左 / R=右 / S=跳过]: ");
                if (a == "L" || a == "l")
                {
                    left_sn = d.serial_number;
                    break;
                }
                if (a == "R" || a == "r")
                {
                    right_sn = d.serial_number;
                    break;
                }
                if (a == "S" || a == "s")
                {
                    break;
                }
            }
        }
        if (left_sn.empty() || right_sn.empty())
        {
            std::cout << "  [警告] 左或右未绑定完整, 不写入相机 JSON"
                      << std::endl;
        }
        else if (left_sn == right_sn)
        {
            std::cout << "  [错误] 左右绑定了同一台相机, 不写入" << std::endl;
            left_sn.clear();
            right_sn.clear();
            ++failures;
        }
        else
        {
            std::cout << "  绑定结果: 左=" << left_sn << "  右=" << right_sn
                      << std::endl;
        }
        }
    }

    // ============ 3. 红外(UVC/V4L2) ============
    std::cout << "\n[3/4] 红外热像(UVC 直插即用)..." << std::endl;
    const auto videos = list_video_devices();
    std::string ir_path;
    if (videos.empty())
    {
        std::cout << "  [错误] 未发现任何 /dev/video*, 检查 USB 线与 dmesg"
                  << std::endl;
        ++failures;
    }
    print_aux_usb_topology("红外", videos);
    if (!check_only && !videos.empty())
    {
        std::cout << "  注: 本机内置摄像头也在候选里(画面是普通彩色图像"
                     "就不是红外)。" << std::endl;
        for (const auto & v : videos)
        {
            if (ask_yes("  预览 " + v + " ?", true))
            {
                if (preview_ir(v))
                {
                    ir_path = v;
                    std::cout << "  红外确认: " << v << std::endl;
                    break;
                }
            }
        }
        if (ir_path.empty())
        {
            std::cout << "  [警告] 红外未确认" << std::endl;
            ++failures;
        }
    }

    // ============ 4. 传感器(Modbus RTU) ============
    std::cout << "\n[4/4] 塔石温湿度/CO2 传感器(Modbus RTU, 9600 8N1)..."
              << std::endl;
    const auto serials = list_serial_devices();
    std::string sensor_path;
    if (serials.empty())
    {
        std::cout << "  [错误] 未发现 /dev/ttyUSB* 或 /dev/ttyACM*;"
                     " 检查 USB 转 485 与驱动(lsusb)" << std::endl;
        ++failures;
    }
    print_aux_usb_topology("传感器", serials);
    if (!check_only && !serials.empty())
    {
        for (const auto & p : serials)
        {
            SerialPort port;
            if (!port.open(p))
            {
                std::cout << "  " << p << " 打开失败(被占用/权限), 跳过"
                          << std::endl;
                std::cout << "  [提示] 权限问题把用户加入 dialout 组后重新"
                             "登录: sudo usermod -aG dialout $USER"
                          << std::endl;
                std::cout << "  [提示] 新装系统若 lsusb 可见 USB 串口但总是"
                             "打不开: 多为 brltty(盲文服务, 会抢占 CH340)或"
                             " ModemManager 所致, 可 sudo systemctl mask --now"
                             " brltty brltty-udev ModemManager 后重新插拔"
                          << std::endl;
                continue;
            }
            std::vector<uint16_t> th;
            if (!port.read_holding(0x01, 0x0000, 2, &th))
            {
                std::cout << "  " << p << " 无有效应答, 试下一个候选..."
                          << std::endl;
                continue;
            }
            std::cout << "  " << p << " 有应答! 实时数据(约 6 秒):"
                      << std::endl;
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::seconds(6);
            while (std::chrono::steady_clock::now() < deadline)
            {
                std::vector<uint16_t> v;
                if (port.read_holding(0x01, 0x0000, 2, &v) && v.size() == 2)
                {
                    std::vector<uint16_t> co2;
                    port.read_holding(0x01, 0x0005, 1, &co2);
                    std::cout << "    温度="
                              << static_cast<int16_t>(v[1]) / 10.0
                              << "C  湿度=" << v[0] / 10.0 << "%RH  CO2="
                              << (co2.empty() ? 0 : co2[0]) << "ppm"
                              << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(900));
            }
            if (ask_yes("  " + p + " 数据正常吗?", true))
            {
                sensor_path = p;
                break;
            }
        }
        if (sensor_path.empty())
        {
            std::cout << "  [警告] 传感器未确认" << std::endl;
            ++failures;
        }
    }

    // ============ 写配置 ============
    if (!check_only)
    {
        std::cout << "\n[写配置] (被修改文件自动备份为 *.bak)" << std::endl;
        if (tuned.fps > 0.0)
        {
            write_stream_tune(root, tuned);
        }
        if (!left_sn.empty())
        {
            patch_json_string_value(
                root + "/config/galaxy_camera_1.json", "serial_number",
                left_sn);
        }
        if (!right_sn.empty())
        {
            patch_json_string_value(
                root + "/config/galaxy_camera_2.json", "serial_number",
                right_sn);
        }
        if (!ir_path.empty())
        {
            patch_json_string_value(
                root + "/config/infrared_camera.json", "device_path",
                ir_path);
        }
        if (!sensor_path.empty())
        {
            // sensor_driver.yaml 的 device_name 键
            const std::string yaml_path =
                root + "/src/tas_sensor_driver/config/sensor_driver.yaml";
            std::string yaml = read_text(yaml_path);
            const std::string needle = "device_name:";
            const size_t pos = yaml.find(needle);
            if (!yaml.empty() && pos != std::string::npos)
            {
                const size_t line_end = yaml.find('\n', pos);
                const std::string old_line =
                    yaml.substr(pos, line_end - pos);
                std::error_code ec;
                fs::copy_file(yaml_path, yaml_path + ".bak",
                              fs::copy_options::overwrite_existing, ec);
                yaml.replace(pos, line_end - pos,
                             needle + " \"" + sensor_path + "\"");
                std::ofstream f(yaml_path, std::ios::trunc);
                f << yaml;
                std::cout << "  [已写入] sensor_driver.yaml: " << old_line
                          << " -> device_name: \"" << sensor_path << "\""
                          << std::endl;
            }
        }
        // 四模块全部确认 => 切真机模式
        const bool all_ok = !left_sn.empty() && !right_sn.empty() &&
                            !ir_path.empty() && !sensor_path.empty();
        if (all_ok)
        {
            std::cout << "\n  四模块全部确认 => launch.json sim_mode 置 false"
                      << std::endl;
            std::string lj = read_text(root + "/config/launch.json");
            const std::string needle = "\"sim_mode\": true";
            const size_t pos = lj.find(needle);
            if (pos != std::string::npos)
            {
                std::error_code ec;
                fs::copy_file(root + "/config/launch.json",
                              root + "/config/launch.json.bak",
                              fs::copy_options::overwrite_existing, ec);
                lj.replace(pos, needle.size(), "\"sim_mode\": false");
                std::ofstream f(root + "/config/launch.json",
                                std::ios::trunc);
                f << lj;
                std::cout << "  [已写入] launch.json: sim_mode=false "
                             "(下次 ./run.sh 即真机模式)" << std::endl;
            }
        }
        else
        {
            std::cout << "\n  存在未确认模块, sim_mode 保持不变; "
                        "之后可再次运行向导继续配置" << std::endl;
        }
    }

    std::cout << "\n==============================================" << std::endl;
    if (failures == 0)
    {
        std::cout << " 检查全部通过 ✓" << std::endl;
    }
    else
    {
        std::cout << " 存在 " << failures << " 项未通过, 见上方排障清单"
                  << std::endl;
    }
    std::cout << "==============================================" << std::endl;
    return failures == 0 ? 0 : 1;
}
