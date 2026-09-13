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
//   5. 全部通过后把 launch.json 的 sim_mode 置 false(真机模式),
//      所有被修改的文件先备份为 *.bak。
//
// 用法(在本机 source 过 install 环境的终端直接运行, 不依赖 ROS 运行时):
//   ros2 run galaxy_camera_dual camera_setup_wizard          # 交互式
//   ros2 run galaxy_camera_dual camera_setup_wizard --check  # 只检查不写入

#include "galaxy_camera_dual/galaxy_device.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <DxImageProc.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
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
// 预览
// ----------------------------------------------------------------------

/// 大恒预览: 打开指定序列号相机, 全屏显示直到按 ENTER
void preview_daheng(const std::string & sn)
{
    // 库生命周期配对: 第 1 步枚举后已 closeLibrary(), 预览前必须重新
    // init(GalaxyDevice::open 不会自动初始化库, 否则 GXUpdateAllDeviceList
    // 直接失败)。
    if (!GalaxyDevice::initLibrary())
    {
        std::cout << "  [错误] Galaxy SDK 初始化失败, 无法预览" << std::endl;
        return;
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
        return;
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
        return;
    }
    const std::string win =
        "preview - SN " + sn + "  (press ENTER to finish)";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::setWindowProperty(win, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    std::cout << "  预览中: SN " << sn << "。目视判断画面朝向, "
              << "然后在预览窗口按 ENTER 关闭..." << std::endl;
    while (true)
    {
        cv::Mat bgr;
        const GX_STATUS st = dev.grab(&frame, 500);
        if (st == GX_STATUS_SUCCESS && frame.nStatus == GX_FRAME_STATUS_SUCCESS)
        {
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
        if (bgr.empty())
        {
            bgr = cv::Mat(960, 1280, CV_8UC3, cv::Scalar(30, 30, 30));
            cv::putText(bgr, "no frame from SN " + sn, cv::Point(40, 480),
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2);
        }
        cv::putText(bgr, "SN " + sn + "  |  press ENTER", cv::Point(30, 70),
                    cv::FONT_HERSHEY_SIMPLEX, 1.4, cv::Scalar(0, 255, 255), 2);
        cv::imshow(win, bgr);
        if (cv::waitKey(30) == 13)   // ENTER
        {
            break;
        }
    }
    cv::destroyWindow(win);
    dev.stopAcquisition();
    dev.close();
    GalaxyDevice::closeLibrary();
}

/// 红外(V4L2)预览: 返回 true = 用户确认画面是热像且正常
bool preview_ir(const std::string & path)
{
    cv::VideoCapture cap(path, cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 512);
    const std::string win = "preview - IR " + path + "  (y=OK n=next)";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    std::cout << "  预览中: " << path << "。画面是热像且正常 => 按 y 确认; "
              << "否则按 n 换下一个候选..." << std::endl;
    while (true)
    {
        cv::Mat frame;
        if (cap.read(frame) && !frame.empty())
        {
            cv::putText(frame, path + "  |  y=OK  n=next",
                        cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                        cv::Scalar(0, 255, 255), 2);
            cv::imshow(win, frame);
        }
        const int key = cv::waitKey(30);
        if (key == 'y' || key == 'Y')
        {
            break;
        }
        if (key == 'n' || key == 'N' || key == 27)
        {
            cv::destroyWindow(win);
            cap.release();
            return false;
        }
    }
    cv::destroyWindow(win);
    cap.release();
    return true;
}

}  // namespace

// ======================================================================

int main(int argc, char ** argv)
{
    const bool check_only =
        (argc > 1 && (std::string(argv[1]) == "--check" ||
                      std::string(argv[1]) == "-c"));
    const std::string root = find_project_root();
    std::cout << "==============================================" << std::endl;
    std::cout << " 涵洞巡检 相机/传感器 配置向导" << std::endl;
    std::cout << "   工程根: " << (root.empty() ? "<未找到!>" : root) << std::endl;
    std::cout << "   模式  : "
              << (check_only ? "只检查(--check)" : "交互式配置") << std::endl;
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
                  << " MAC=" << d.mac << std::endl;
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

    // ============ 2. 左右相机确认与绑定 ============
    std::string left_sn;
    std::string right_sn;
    if (!check_only && cams.size() >= 2)
    {
        std::cout << "\n[2/4] 左右相机确认(逐台全屏预览, 看画面回答是哪侧)..."
                  << std::endl;
        for (const auto & d : cams)
        {
            preview_daheng(d.serial_number);
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
    for (const auto & v : videos)
    {
        std::cout << "  候选: " << v << std::endl;
    }
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
    for (const auto & p : serials)
    {
        std::cout << "  候选: " << p << std::endl;
    }
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
        if (!left_sn.empty())
        {
            patch_json_string_value(
                root + "/config/galaxy_camera1_.json", "serial_number",
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
