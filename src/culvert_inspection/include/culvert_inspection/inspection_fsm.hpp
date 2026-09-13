// 文件: inspection_fsm.hpp
// 作用: 巡检协议状态机核心(无 ROS 依赖, 可被 gtest 直接单测)。
//
// 协议(std_msgs/msg/UInt8, 详见 inspection_node):
//   输入 0x01 -> 记录触发时刻 t(单调钟), 左右相机以"触发时刻的到达计数"
//                为基线, 各自独立等待"t 后按到达顺序的第 N 帧"(N 默认 2,
//                t 时刻视为第 0 帧);
//   目标帧解码失败 -> 顺延取其后第 N+1、N+2…帧(不计为进展);
//   自最近一次成功捕获(或触发)起超过 frame_timeout_sec -> 放弃
//   (不发 0x02), 持续坏帧同样会被超时兜住;
//   双帧齐 -> 写 <左帧时间戳>_left.jpg / <右帧时间戳>_right.jpg;
//             全部成功 -> 恰好发一次 0x02; 任一失败 -> 不发 0x02。
//
// 线程约定(由 inspection_node 外部保证):
//   - onFrame 的"空闲快路径"(仅到达计数)是无锁原子的;
//   - 其余调用(onCommand/onFrame 全量/checkTimeout)必须外部串行化
//     (节点用互斥锁; 指令侧用 try_lock 实现"处理中再收 0x01 忽略");
//   - 解码/写盘/发确认/取时间/打日志全部通过注入接口, 单测可替换。
//
// 状态: Idle(空闲) / Armed(已触发等帧) / Saving(写盘中, 瞬态)。
//       Timeout/Failed 不作为驻留状态(放弃后立即回 Idle), 仅体现在日志。

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace culvert_inspection
{

/// 巡检状态机状态(数值会出现在状态话题里, 供外部断言)。
enum class FsmState : int
{
    kIdle = 0,     ///< 空闲: 等待 0x01
    kArmed = 1,    ///< 已触发: 等待左右目标帧
    kSaving = 2,   ///< 双帧已取到: 正在写盘(瞬态)
};

inline const char * fsmStateName(FsmState s)
{
    switch (s)
    {
    case FsmState::kIdle: return "IDLE";
    case FsmState::kArmed: return "ARMED";
    case FsmState::kSaving: return "SAVING";
    }
    return "UNKNOWN";
}

/// 一帧到达的图像数据(到达顺序由状态机内部计数, 不依赖时间戳)。
struct InspectionFrame
{
    const uint8_t * data = nullptr;   ///< 原始 JPEG 字节(不拷贝)
    size_t size = 0;
    int64_t stamp_ns = 0;             ///< 消息时间戳(纳秒, 用于文件名)
};

/// 巡检状态机(外部串行化调用; Side::arrived 原子计数除外)。
class InspectionFsm
{
public:
    // ---- 注入接口(单测可替换) ----
    using Decoder = std::function<bool(
        const uint8_t * data, size_t size)>;             ///< 帧解码校验
    using Writer = std::function<bool(
        const std::string & path, const uint8_t * data, size_t size)>;  ///< 写盘
    using AckSender = std::function<void()>;             ///< 发布 0x02
    using Remover = std::function<bool(const std::string & path)>;  ///< 删文件
    using Clock = std::function<std::chrono::steady_clock::time_point()>;  ///< 单调钟
    using WallNs = std::function<int64_t()>;             ///< 系统钟纳秒(文件名兜底)
    using Logger = std::function<void(const char * level, const std::string &)>;

    /// 一路相机(左/右)的到达计数与暂存帧。
    struct Side
    {
        std::string name;                     ///< "left" / "right"
        /// 累计到达帧数(只增)。显式 seq_cst: 与 arm() 基线读取构成
        /// 全序, 保证任一帧要么整体在触发前、要么记为 t 后第 1 帧
        /// (见 onFrame 的内存序说明)。
        std::atomic<uint64_t> arrived{0};

        // ---- 以下仅在外部串行化保护下访问 ----
        uint64_t baseline = 0;                ///< 触发时刻的 arrived 快照
        bool captured = false;                ///< 目标帧是否已暂存
        std::vector<uint8_t> jpeg;            ///< 暂存的 JPEG(已通过解码)
        int64_t stamp_ns = 0;                 ///< 该帧时间戳(文件名)

        // ---- 标注图(模型处理完框出来的图, 与原图同名配对保存) ----
        std::vector<uint8_t> ann_jpeg;  ///< 暂存的标注图 JPEG
        int64_t ann_stamp_ns = 0;       ///< 标注图源帧时间戳
        bool ann_captured = false;      ///< 是否已取到时间戳配对的标注图
    };

    struct Params
    {
        int target_frame_index = 2;           ///< t 后第 N 帧(N=2)
        double frame_timeout_sec = 3.0;       ///< 无进展超时(秒)
        std::string save_dir;                 ///< 保存目录
        double annot_gap_ms = 500.0;          ///< 标注图与原图时间戳配对容差
        std::string raw_subdir = "raw";       ///< 原图保存子目录
        std::string annotated_subdir = "annotated";  ///< 标注图保存子目录
    };

    /// Side 含 atomic(不可拷贝/移动), 由状态机内部按名构造;
    /// 通过 left()/right() 取引用供节点回调与单测使用。
    InspectionFsm(Params params, const std::string & left_name,
                  const std::string & right_name, Decoder decoder,
                  Writer writer, Remover remover, AckSender ack, Clock clock,
                  WallNs wall_ns, Logger logger)
    : params_(params), decoder_(std::move(decoder)), writer_(std::move(writer)),
      remover_(std::move(remover)), ack_(std::move(ack)), clock_(std::move(clock)),
      wall_ns_(std::move(wall_ns)), log_(std::move(logger))
    {
        left_.name = left_name;
        right_.name = right_name;
    }

    FsmState state() const { return state_.load(std::memory_order_acquire); }
    uint64_t cycle() const { return cycle_id_; }
    Side & left() { return left_; }     ///< 节点图像回调/单测使用
    Side & right() { return right_; }

    /// 收到指令字节。返回: 1=触发新巡检; 0=忽略(非 Idle 或重复 0x01);
    /// -1=非法数据(非 0x01)。需外部串行化。
    int onCommand(uint8_t data)
    {
        if (data != 0x01)
        {
            log_("WARN", "收到未知巡检指令(仅支持 0x01), 忽略");
            return -1;
        }
        if (state_.load(std::memory_order_acquire) != FsmState::kIdle)
        {
            // 重复 0x01 / 处理中再次 0x01: 节流告警并忽略(调用方 try_lock
            // 一般已拦下 Saving; 这里兜底拦截 Armed)。
            throttle("WARN", "巡检处理中再次收到 0x01, 忽略"
                     "(每个 0x01 只触发一次保存)");
            return 0;
        }
        arm();
        log_("INFO", "收到 0x01: 触发巡检, 等待左右相机 t 后第 " +
             std::to_string(params_.target_frame_index) + " 帧(超时 " +
             std::to_string(params_.frame_timeout_sec) + "s)");
        return 1;
    }

    /// 到达一路标注图(模型处理完框出来的图): 仅在该侧原图已捕获、
    /// 且时间戳与原图配对(容差 annot_gap_ms)时暂存。两侧原图+标注图
    /// 四者齐备才写盘。需外部串行化。
    void onAnnotatedFrame(Side & side, const InspectionFrame & frame)
    {
        if (state_.load(std::memory_order_acquire) != FsmState::kArmed ||
            !side.captured || side.ann_captured)
        {
            return;
        }
        const double gap_ms = std::abs(
            static_cast<double>(frame.stamp_ns - side.stamp_ns)) / 1e6;
        if (frame.stamp_ns <= 0 || gap_ms > params_.annot_gap_ms)
        {
            return;   // 时间戳对不上(模型滞后过多): 继续等后续标注图
        }
        side.ann_jpeg.assign(frame.data, frame.data + frame.size);
        side.ann_stamp_ns = frame.stamp_ns;
        side.ann_captured = true;
        touchProgress();   // 标注图到达也算进展
        log_("INFO", side.name + " 标注图已配对(时间戳 " +
             std::to_string(frame.stamp_ns) + ", 偏差 " +
             std::to_string(gap_ms) + "ms)");
        finishIfReady();
    }

    /// 到达一帧并按状态机处理。
    /// 线程契约: 节点侧用 state_mutex 把本函数与 onCommand/checkTimeout
    /// 完全串行化 —— 计数、arm() 基线读取、Armed 发布三者互斥, 从构造
    /// 上关闭"帧落在 arm 执行中间被跳过"的窗口(无锁快路径曾留下该
    /// 窗口: 帧计数晚于基线读取却观察到旧的 Idle 状态而跳过目标帧)。
    /// 60 帧/秒的量级下每帧取锁开销可忽略, 用正确性换微优化。
    /// 内存序: 计数与基线读取显式 seq_cst, 防止维护者误改弱序。
    void onFrame(Side & side, const InspectionFrame & frame)
    {
        // 到达顺序计数: 无论何种状态都计数(协议以到达顺序为准)。
        side.arrived.fetch_add(1, std::memory_order_seq_cst);

        if (state_.load(std::memory_order_acquire) != FsmState::kArmed)
        {
            return;   // 空闲/写盘中: 只计数
        }
        const uint64_t index_after_t =
            side.arrived.load(std::memory_order_seq_cst) - side.baseline;
        if (index_after_t < static_cast<uint64_t>(params_.target_frame_index))
        {
            return;   // 目标帧之前的帧
        }
        if (side.captured)
        {
            return;   // 目标帧已到手, 后续帧不再需要
        }

        // 帧解码校验: 失败即作废该帧, 继续等其后的第 N+1、N+2…帧。
        // 注意: 解码失败"不刷新"进展时刻 —— 持续坏帧也会在超时后放弃,
        // 不会无限顺延(超时 = 自最近一次成功捕获/触发起算)。
        if (!decoder_(frame.data, frame.size))
        {
            throttle("WARN", side.name + " 相机 t 后第 " +
                     std::to_string(index_after_t) + " 帧解码失败, 顺延等待");
            return;
        }

        side.jpeg.assign(frame.data, frame.data + frame.size);
        side.stamp_ns = frame.stamp_ns;
        side.captured = true;
        touchProgress();
        log_("INFO", side.name + " 相机已取到 t 后第 " +
             std::to_string(index_after_t) + " 帧(时间戳 " +
             std::to_string(frame.stamp_ns) + ")");
        finishIfReady();
    }

    /// 周期调用(看门狗): 无进展超时则放弃当前周期(不发 0x02)。
    /// 需外部串行化。
    void checkTimeout()
    {
        if (state_.load(std::memory_order_acquire) != FsmState::kArmed)
        {
            return;
        }
        const auto now = clock_();
        if (std::chrono::duration<double>(now - last_progress_).count() >
            params_.frame_timeout_sec)
        {
            log_("ERROR", "[cycle " + std::to_string(cycle_id_) +
                 "] 等待目标帧超时(" +
                 std::to_string(params_.frame_timeout_sec) + "s 无进展), "
                 "放弃本次巡检, 不发送 0x02");
            state_.store(FsmState::kIdle, std::memory_order_release);
        }
    }

private:
    /// 发起新巡检(Idle 状态下调用)。
    void arm()
    {
        ++cycle_id_;
        trigger_time_ = clock_();
        trigger_ns_ = wall_ns_();
        last_progress_ = trigger_time_;   // 触发本身算一次进展
        left_.baseline = left_.arrived.load();
        right_.baseline = right_.arrived.load();
        left_.captured = right_.captured = false;
        left_.jpeg.clear();
        right_.jpeg.clear();
        state_.store(FsmState::kArmed, std::memory_order_release);
    }

    /// 刷新"最近进展"时刻(超时从此起算)。
    void touchProgress() { last_progress_ = clock_(); }

    /// 左右原图 + 左右标注图四者齐 -> 写盘 -> 成功发一次 0x02。
    /// (需外部串行化; onFrame 与 onAnnotatedFrame 都会调用本函数)
    void finishIfReady()
    {
        if (!left_.captured || !right_.captured ||
            !left_.ann_captured || !right_.ann_captured)
        {
            return;
        }
        state_.store(FsmState::kSaving, std::memory_order_release);

        // 文件名 = 所选帧自身时间戳(纳秒); 异常(<=0)时退化用触发时刻,
        // 避免跨周期互相覆盖。原图与标注图分别存 raw/ 与 annotated/。
        const auto prefix = [this](const Side & s) {
            return s.stamp_ns > 0 ? std::to_string(s.stamp_ns)
                                  : std::to_string(trigger_ns_);
        };
        const std::string raw_dir =
            params_.save_dir + "/" + params_.raw_subdir;
        const std::string ann_dir =
            params_.save_dir + "/" + params_.annotated_subdir;
        const std::string left_path =
            raw_dir + "/" + prefix(left_) + "_left.jpg";
        const std::string right_path =
            raw_dir + "/" + prefix(right_) + "_right.jpg";
        const std::string left_ann_path =
            ann_dir + "/" + prefix(left_) + "_left.jpg";
        const std::string right_ann_path =
            ann_dir + "/" + prefix(right_) + "_right.jpg";

        const bool ok_left =
            writer_(left_path, left_.jpeg.data(), left_.jpeg.size());
        const bool ok_right =
            writer_(right_path, right_.jpeg.data(), right_.jpeg.size());
        const bool ok_left_ann =
            writer_(left_ann_path, left_.ann_jpeg.data(),
                    left_.ann_jpeg.size());
        const bool ok_right_ann =
            writer_(right_ann_path, right_.ann_jpeg.data(),
                    right_.ann_jpeg.size());
        const bool ok = ok_left && ok_right && ok_left_ann && ok_right_ann;

        if (ok)
        {
            ack_();   // 仅此一处发布 0x02
            log_("INFO", "[cycle " + std::to_string(cycle_id_) +
                 "] 巡检完成: 原图 " + left_path + " / " + right_path +
                 " + 标注图 " + left_ann_path + " / " + right_ann_path +
                 " 已写入, 发布确认 0x02");
            state_.store(FsmState::kIdle, std::memory_order_release);
            return;
        }
        // 任一失败: 清理所有已写出的文件, 避免残留不完整数据。
        if (ok_left) { remover_(left_path); }
        if (ok_right) { remover_(right_path); }
        if (ok_left_ann) { remover_(left_ann_path); }
        if (ok_right_ann) { remover_(right_ann_path); }
        log_("ERROR", "[cycle " + std::to_string(cycle_id_) + "] 写盘失败(" +
             std::string(ok_left ? "left ok" : "left FAIL") + "/" +
             std::string(ok_right ? "right ok" : "right FAIL") +
             "), 已清理半对文件, 不发送 0x02");
        state_.store(FsmState::kIdle, std::memory_order_release);
    }

    /// 同文案节流日志(2s), 防止异常输入刷屏。
    void throttle(const char * level, const std::string & text)
    {
        const auto now = clock_();
        if (text != throttle_text_ ||
            std::chrono::duration<double>(now - throttle_time_).count() > 2.0)
        {
            log_(level, text);
            throttle_text_ = text;
            throttle_time_ = now;
        }
    }

    Params params_;
    Side left_;
    Side right_;
    Decoder decoder_;
    Writer writer_;
    Remover remover_;
    AckSender ack_;
    Clock clock_;
    WallNs wall_ns_;
    Logger log_;

    std::atomic<FsmState> state_{FsmState::kIdle};
    uint64_t cycle_id_ = 0;
    std::chrono::steady_clock::time_point trigger_time_{};  ///< t
    std::chrono::steady_clock::time_point last_progress_{}; ///< 最近进展
    int64_t trigger_ns_ = 0;   ///< 触发时刻(系统钟 ns, 文件名兜底)
    std::string throttle_text_;
    std::chrono::steady_clock::time_point throttle_time_{};
};

}  // namespace culvert_inspection
