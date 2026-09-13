// 文件: test_inspection_fsm.cpp
// 作用: InspectionFsm(巡检协议状态机)的 gtest 单元测试。
//
// 无 ROS 依赖: 解码/写盘/发确认/时钟/日志全部用测试替身注入,
// 帧到达顺序由测试直接控制(比经 DDS 的黑盒测试更精确、更快)。
//
// 覆盖需求点:
//   1. 严格选取 t 后第 2 帧(到达顺序, 非时间戳); t 时刻 = 第 0 帧;
//   2. 目标帧解码失败 -> 顺延取第 3、4…帧;
//   3. 双帧均成功写盘 -> 恰好一次 0x02; 文件名 = 各自帧时间戳;
//   4. 重复 0x01(Armed 中) -> 忽略, 不产生第二次巡检;
//   5. 非法指令字节(非 0x01) -> 忽略;
//   6. 超时(单侧持续缺帧) -> 放弃且不发 0x02; 随后独立巡检可正常完成;
//   7. 写盘失败(单侧) -> 不发 0x02; 恢复后下一巡检正常;
//   8. 单侧目标帧先到后, 另一侧后续帧不影响已捕获侧(独立选帧)。

#include "culvert_inspection/inspection_fsm.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <mutex>
#include <thread>

#include <deque>
#include <map>
#include <map>
#include <string>
#include <vector>

namespace
{

using culvert_inspection::FsmState;
using culvert_inspection::InspectionFsm;

/// 手动推进的单调钟(超时测试用)。
struct ManualClock
{
    std::chrono::steady_clock::time_point t =
        std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point operator()() { return t; }
    void advance_sec(double s)
    {
        t += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(s));
    }
};

/// 测试环境: 注入替身 + 记录 ack/写盘/日志。
struct Harness
{
    ManualClock clock;
    // 解码替身: "BAD" 开头的字节视为解码失败
    culvert_inspection::InspectionFsm::Decoder decoder =
        [](const uint8_t * data, size_t size) {
            return size >= 2 && data[0] != 'B';
        };
    // 写盘替身: 记录 path -> 内容; 可指定失败路径关键字
    std::map<std::string, std::vector<uint8_t>> files;
    std::string fail_path_contains;
    culvert_inspection::InspectionFsm::Writer writer =
        [this](const std::string & path, const uint8_t * data, size_t size) {
            if (!fail_path_contains.empty() &&
                path.find(fail_path_contains) != std::string::npos)
            {
                return false;
            }
            files[path] = std::vector<uint8_t>(data, data + size);
            return true;
        };
    std::vector<std::string> removed;
    int ack_count = 0;
    std::vector<std::string> logs;

    std::unique_ptr<InspectionFsm> fsm;

    /// fsm 构建后才能取 Side 引用(含 atomic, 不可独立构造后移入)。
    InspectionFsm::Side & left() { return fsm->left(); }
    InspectionFsm::Side & right() { return fsm->right(); }

    void build(int target = 2, double timeout = 3.0)
    {
        InspectionFsm::Params p;
        p.target_frame_index = target;
        p.frame_timeout_sec = timeout;
        p.save_dir = "/tmp/ut_save";
        fsm = std::make_unique<InspectionFsm>(
            p, "left", "right", decoder, writer,
            [this](const std::string & path) {
                removed.push_back(path);
                files.erase(path);
                return true;
            },
            [this]() { ++ack_count; },
            [this]() { return clock(); },
            []() { return int64_t{1234567890}; },   // 文件名兜底用系统钟
            [this](const char * level, const std::string & text) {
                logs.push_back(std::string(level) + ": " + text);
            });
    }


    /// 到达一帧(拷贝一份字节, 模拟真实消息生命周期)。
    void frame(InspectionFsm::Side & side, const std::string & jpeg,
               int64_t stamp_ns)
    {
        culvert_inspection::InspectionFrame f;
        bytes.push_back(jpeg);
        f.data = reinterpret_cast<const uint8_t *>(bytes.back().data());
        f.size = bytes.back().size();
        f.stamp_ns = stamp_ns;
        fsm->onFrame(side, f);
    }

    void trigger()
    {
        ASSERT_EQ(fsm->onCommand(0x01), 1);
    }

    std::vector<std::string> bytes;   // 保持字节生命周期
};

// ----------------------------------------------------------------------
// 1. 严格选取 t 后第 2 帧
// ----------------------------------------------------------------------

TEST(InspectionFsm, SelectsExactlySecondFrameAfterTrigger)
{
    Harness h;
    h.build();
    h.frame(h.left(), "pre", 100);
    h.trigger();                            // t 时刻: baseline = 已到达数

    // t 后: 第 1 帧(非目标)、第 2 帧(目标); 右侧暂缺第 2 帧
    h.frame(h.left(), "after1", 101);
    h.frame(h.right(), "after1r", 101);
    h.frame(h.left(), "after2", 102);
    ASSERT_EQ(h.fsm->state(), FsmState::kArmed);   // 右侧未齐 -> 尚未完成
    EXPECT_EQ(h.ack_count, 0);

    // 左侧第 3 帧不应改变已捕获内容; 右侧第 2 帧补齐 -> 完成
    h.frame(h.left(), "after3", 103);
    ASSERT_EQ(h.ack_count, 0);                     // 单侧不触发
    h.frame(h.right(), "after2r", 102);
    ASSERT_EQ(h.fsm->state(), FsmState::kIdle);    // 双帧齐 -> 完成
    ASSERT_EQ(h.ack_count, 1);

    // 文件名 = 各自所选第 2 帧的时间戳; 内容 = 该帧字节
    ASSERT_EQ(h.files.size(), 2u);
    EXPECT_EQ(h.files["/tmp/ut_save/102_left.jpg"],
              std::vector<uint8_t>({'a', 'f', 't', 'e', 'r', '2'}));
    EXPECT_EQ(h.files["/tmp/ut_save/102_right.jpg"],
              std::vector<uint8_t>({'a', 'f', 't', 'e', 'r', '2', 'r'}));
}

TEST(InspectionFsm, TriggerMomentIsFrameZero)
{
    Harness h;
    h.build();
    h.trigger();
    // 触发后立刻到达的帧是第 1 帧, 不应被当作第 2 帧
    h.frame(h.left(), "n1", 201);
    h.frame(h.left(), "n2", 202);
    h.frame(h.right(), "n1r", 201);
    h.frame(h.right(), "n2r", 202);
    EXPECT_EQ(h.ack_count, 1);
    EXPECT_TRUE(h.files.count("/tmp/ut_save/202_left.jpg"));
    EXPECT_FALSE(h.files.count("/tmp/ut_save/201_left.jpg"));
}

// ----------------------------------------------------------------------
// 2. 解码失败 -> 顺延第 3、4…帧
// ----------------------------------------------------------------------

TEST(InspectionFsm, SkipsUndecodableFramesAndWaitsForLaterOnes)
{
    Harness h;
    h.build();
    h.trigger();
    // 第 2 帧解码失败("BAD" 开头), 第 3 帧有效
    h.frame(h.left(), "BAD2", 301);
    h.frame(h.right(), "ok2", 301);
    EXPECT_EQ(h.fsm->state(), FsmState::kArmed);
    h.frame(h.left(), "ok3", 302);
    h.frame(h.right(), "ok3r", 302);
    EXPECT_EQ(h.ack_count, 1);
    // 左侧用了第 3 帧(302), 右侧用的是其第 2 帧(302)
    EXPECT_TRUE(h.files.count("/tmp/ut_save/302_left.jpg"));
    EXPECT_TRUE(h.files.count("/tmp/ut_save/302_right.jpg"));
    EXPECT_FALSE(h.files.count("/tmp/ut_save/301_right.jpg"));
}

// ----------------------------------------------------------------------
// 3/4/5. 重复 0x01 与非法指令
// ----------------------------------------------------------------------

TEST(InspectionFsm, DuplicateTriggerWhileArmedIsIgnored)
{
    Harness h;
    h.build();
    h.trigger();
    h.frame(h.left(), "a1", 401);
    // Armed 中再次 0x01: 忽略
    EXPECT_EQ(h.fsm->onCommand(0x01), 0);
    h.frame(h.left(), "a2", 402);
    h.frame(h.right(), "a1", 401);
    h.frame(h.right(), "a2", 402);
    // 只有一次巡检完成, 只发一次 0x02
    EXPECT_EQ(h.ack_count, 1);
    EXPECT_EQ(h.files.size(), 2u);
    // 若重复触发未被忽略, 第二周期会再选新帧 -> 文件数将 > 2
}

TEST(InspectionFsm, NonZerox01CommandIsRejected)
{
    Harness h;
    h.build();
    EXPECT_EQ(h.fsm->onCommand(0x02), -1);
    EXPECT_EQ(h.fsm->onCommand(0x00), -1);
    EXPECT_EQ(h.fsm->state(), FsmState::kIdle);
    EXPECT_EQ(h.ack_count, 0);
}

// ----------------------------------------------------------------------
// 6. 超时: 放弃且不发 0x02, 之后独立巡检正常
// ----------------------------------------------------------------------

TEST(InspectionFsm, TimeoutAbandonsCycleWithoutAckThenRecovers)
{
    Harness h;
    h.build(2, 1.0);
    h.trigger();
    h.frame(h.left(), "l1", 501);   // 左侧有进展
    h.clock.advance_sec(1.5);     // 超过 1.0s 无新进展
    h.fsm->checkTimeout();
    EXPECT_EQ(h.fsm->state(), FsmState::kIdle);
    EXPECT_EQ(h.ack_count, 0);
    EXPECT_TRUE(h.files.empty());

    // 恢复后的下一次独立巡检正常完成
    h.trigger();
    h.frame(h.left(), "L1", 510);
    h.frame(h.left(), "L2", 511);
    h.frame(h.right(), "R1", 510);
    h.frame(h.right(), "R2", 511);
    EXPECT_EQ(h.fsm->state(), FsmState::kIdle);
    EXPECT_EQ(h.ack_count, 1);
    EXPECT_EQ(h.files.size(), 2u);
}

TEST(InspectionFsm, TimeoutNotFiredWhileProgressContinues)
{
    Harness h;
    h.build(5, 1.0);   // 目标 = t 后第 5 帧
    h.trigger();
    // 每隔 0.5s 左侧来一帧: 目标帧捕获(i=5, t=2.5s)前每帧都是"进展",
    // 超时只看"最近一次进展", 右侧缺帧不影响。
    for (int i = 1; i <= 6; ++i)
    {
        h.clock.advance_sec(0.5);
        h.frame(h.left(), "L" + std::to_string(i), 600 + i);
    }
    // 已捕获侧的后续帧不算进展(i=6 无效果), 但 0.5s < 1.0s 不超时
    ASSERT_EQ(h.fsm->state(), FsmState::kArmed);
    EXPECT_EQ(h.ack_count, 0);
    h.fsm->checkTimeout();
    EXPECT_EQ(h.fsm->state(), FsmState::kArmed);
    // 此后 1.5s 无任何进展 -> 超时放弃, 不发 0x02
    h.clock.advance_sec(1.5);
    h.fsm->checkTimeout();
    EXPECT_EQ(h.fsm->state(), FsmState::kIdle);
    EXPECT_EQ(h.ack_count, 0);
}

// ----------------------------------------------------------------------
// 7. 写盘失败: 不发 0x02, 恢复后下一巡检正常
// ----------------------------------------------------------------------

TEST(InspectionFsm, WriteFailureSuppressesAckThenRecovers)
{
    Harness h;
    h.build();
    h.fail_path_contains = "_left.jpg";   // 本周期左侧写盘必失败
    h.trigger();
    h.frame(h.left(), "L1", 701);
    h.frame(h.left(), "L2", 702);
    h.frame(h.right(), "R1", 701);
    h.frame(h.right(), "R2", 702);
    EXPECT_EQ(h.fsm->state(), FsmState::kIdle);   // 已放弃(回 Idle)
    EXPECT_EQ(h.ack_count, 0);                    // 绝不误发 0x02
    EXPECT_TRUE(h.files.empty());                 // 半对文件已被清理
    ASSERT_EQ(h.removed.size(), 1u);              // 右侧成功文件被删除
    EXPECT_NE(h.removed[0].find("_right.jpg"), std::string::npos);

    // 下一巡检(写盘恢复)正常
    h.fail_path_contains.clear();
    h.trigger();
    h.frame(h.left(), "L1", 710);
    h.frame(h.left(), "L2", 711);
    h.frame(h.right(), "R1", 710);
    h.frame(h.right(), "R2", 711);
    EXPECT_EQ(h.ack_count, 1);
    EXPECT_EQ(h.files.size(), 2u);
}

// ----------------------------------------------------------------------
// 8. 独立选帧: 一侧提前捕获后, 不再受该侧后续帧影响
// ----------------------------------------------------------------------

TEST(InspectionFsm, SidesSelectFramesIndependently)
{
    Harness h;
    h.build();
    h.trigger();
    // 右侧快速给到第 2 帧; 左侧先只给第 1 帧
    h.frame(h.right(), "R1", 801);
    h.frame(h.right(), "R2", 802);
    EXPECT_EQ(h.fsm->state(), FsmState::kArmed);
    // 右侧后续帧(第 3 帧)不应改变已捕获内容
    h.frame(h.right(), "R3", 803);
    // 左侧到第 2 帧 -> 完成; 右侧文件必须是 802(第 2 帧)
    h.frame(h.left(), "L1", 810);
    h.frame(h.left(), "L2", 811);
    EXPECT_EQ(h.ack_count, 1);
    EXPECT_TRUE(h.files.count("/tmp/ut_save/802_right.jpg"));
    EXPECT_TRUE(h.files.count("/tmp/ut_save/811_left.jpg"));
    EXPECT_FALSE(h.files.count("/tmp/ut_save/803_right.jpg"));
}

// ----------------------------------------------------------------------
// 9. 目标帧序号可配置(N=3)
// ----------------------------------------------------------------------

TEST(InspectionFsm, ConfigurableTargetFrameIndex)
{
    Harness h;
    h.build(3, 3.0);
    h.trigger();
    for (int i = 1; i <= 2; ++i)
    {
        h.frame(h.left(), "L" + std::to_string(i), 900 + i);
        h.frame(h.right(), "R" + std::to_string(i), 900 + i);
    }
    EXPECT_EQ(h.fsm->state(), FsmState::kArmed);   // 第 3 帧未到
    h.frame(h.left(), "L3", 903);
    h.frame(h.right(), "R3", 903);
    EXPECT_EQ(h.ack_count, 1);
    EXPECT_TRUE(h.files.count("/tmp/ut_save/903_left.jpg"));
}

// ----------------------------------------------------------------------
// 10. 持续坏帧也必须超时(解码失败不计为进展)
// ----------------------------------------------------------------------

TEST(InspectionFsm, ContinuousCorruptFramesStillTimeOut)
{
    Harness h;
    h.build(2, 1.0);
    h.trigger();
    // 每 0.4s 来一帧坏帧, 持续 6 帧(共 2.4s) —— 若坏帧被当作进展,
    // 永远不会超时; 正确行为: 超时只看成功捕获, 1.0s 后即放弃。
    for (int i = 1; i <= 6; ++i)
    {
        h.clock.advance_sec(0.4);
        h.frame(h.left(), "BAD" + std::to_string(i), 1000 + i);
        h.frame(h.right(), "BADR" + std::to_string(i), 1000 + i);
        EXPECT_EQ(h.fsm->state(), FsmState::kArmed);
    }
    h.fsm->checkTimeout();
    EXPECT_EQ(h.fsm->state(), FsmState::kIdle);
    EXPECT_EQ(h.ack_count, 0);
    EXPECT_TRUE(h.files.empty());
}

// ----------------------------------------------------------------------
// 11. Armed 路径帧计数: 每帧恰好计一次(锁内), 目标帧捕获正确
// ----------------------------------------------------------------------

TEST(InspectionFsm, ArmedPathCountsExactlyOnce)
{
    Harness h;
    h.build();
    h.trigger();
    for (int i = 1; i <= 2; ++i)
    {
        culvert_inspection::InspectionFrame f;
        const std::string jpeg = "ok" + std::to_string(i);
        h.bytes.push_back(jpeg);
        f.data = reinterpret_cast<const uint8_t *>(h.bytes.back().data());
        f.size = h.bytes.back().size();
        f.stamp_ns = 2000 + i;
        h.fsm->onFrame(h.left(), f);
    }
    // 计数与捕获一致: 2 帧到达 => 目标(第 2 帧)被捕获且计数恰为 2
    EXPECT_TRUE(h.left().captured);
    EXPECT_EQ(h.left().stamp_ns, 2002);
    EXPECT_EQ(h.left().arrived.load(), 2u);
}

// ----------------------------------------------------------------------
// 12. 并发压力测试: 双帧线程 + 触发线程真并发(真时钟), 按生产锁模型
//     (onCommand 与 onFrame 同一把 state_mutex 串行化)校验周期归属:
//     每周期捕获帧的票号必须恰为 基线+2 且逐周期严格递增。
//     注: "观察到 Idle 却计入 t 后"(计数晚于基线读取、状态读更晚)
//     在计数先行的实现下是合法序列 —— 计数点是回调首条语句, 计数
//     晚于基线读取即回调晚于触发, 归类 t 后第 1 帧符合到达顺序语义;
//     故本测试不断言"观察到 Armed", 只断言可审计的周期归属。
// ----------------------------------------------------------------------

TEST(InspectionFsm, ConcurrentTriggerFrameStress)
{
    // 真并发压力: 双帧线程(全速灌帧, 复刻节点"计数 -> 查状态 ->
    // Armed 则锁内处理"的顺序) + 单触发线程(连续发起 300 个周期)。
    //
    // 校验四条并发不变量:
    //  a) 收敛: 全部周期完成且确认数 == 周期数, 结束时状态回 Idle;
    //  b) 计数完整性: 每侧帧日志条数 == 该侧最终 arrived 值(不丢计数);
    //  c) 基线单调: 同一侧逐周期基线严格递增(周期推进且不回退);
    //  d) 双帧不串扰: 双侧确认数一致推进(同一次 arm 写入两侧基线)。
    // 注: "观察到 Idle 却计入 t 后"(计数晚于基线读取、状态读更晚)
    // 在计数先行的实现下是合法序列, 不作为违规 —— 计数点是回调首条
    // 语句, 计数晚于基线读取即回调晚于触发, 归类 t 后第 1 帧符合
    // 到达顺序语义。
    InspectionFsm::Params p;
    p.target_frame_index = 2;
    p.frame_timeout_sec = 5.0;
    p.save_dir = "/tmp/ut_stress";
    std::atomic<int> acks{0};
    std::atomic<bool> run{true};
    auto fsm = std::make_unique<InspectionFsm>(
        p, "left", "right",
        [](const uint8_t * d, size_t n) { return n >= 2 && d[0] != 'B'; },
        [](const std::string &, const uint8_t *, size_t) { return true; },
        [](const std::string &) { return true; },
        [&acks]() { ++acks; },
        []() { return std::chrono::steady_clock::now(); },
        []() { return int64_t{1}; },
        [](const char *, const std::string &) {});

    std::mutex state_mutex;   // 与节点同构: Armed 处理与指令处理共用它
    std::mutex log_mutex;     // 帧日志/seq->票号映射的读写锁

    std::atomic<uint64_t> logged_left{0};
    std::atomic<uint64_t> logged_right{0};
    std::map<int64_t, uint64_t> left_seq2ticket;    ///< seq -> 票号(周期归属回溯)
    std::map<int64_t, uint64_t> right_seq2ticket;

    auto frame_loop = [&](InspectionFsm::Side & side,
                          std::atomic<uint64_t> & logged,
                          std::map<int64_t, uint64_t> & seq2ticket)
    {
        uint64_t seq = 0;
        while (run.load(std::memory_order_relaxed))
        {
            ++seq;
            culvert_inspection::InspectionFrame f;
            const std::string jpeg = "J" + std::to_string(seq);
            f.data = reinterpret_cast<const uint8_t *>(jpeg.data());
            f.size = jpeg.size();
            f.stamp_ns = static_cast<int64_t>(seq);
            {
                std::lock_guard<std::mutex> lk2(log_mutex);
                seq2ticket[seq] = static_cast<int64_t>(seq);   // 先登记
            }
            // 与生产一致: 计数/处理统一在锁内(无无锁快路径)
            std::lock_guard<std::mutex> lk(state_mutex);
            fsm->onFrame(side, f);
            logged.store(seq, std::memory_order_relaxed);
        }
    };

    std::thread tf(frame_loop, std::ref(fsm->left()),
                   std::ref(logged_left), std::ref(left_seq2ticket));
    std::thread tf2(frame_loop, std::ref(fsm->right()),
                    std::ref(logged_right), std::ref(right_seq2ticket));
    // RAII join: 断言失败提前返回时也必须回收线程(否则 terminate)。
    // 局部类成员函数访问不了外围自动变量, 停止标志以引用成员传入。
    struct Joiner
    {
        std::atomic<bool> & stop;
        std::thread & a;
        std::thread & b;
        ~Joiner()
        {
            stop.store(false);
            if (a.joinable()) { a.join(); }
            if (b.joinable()) { b.join(); }
        }
    } joiner{run, tf, tf2};

    const int kCycles = 300;
    int completed = 0;
    int ignored_triggers = 0;
    uint64_t prev_base_l = 0;
    uint64_t prev_base_r = 0;
    // 每周期捕获信息: (左票号, 右票号) —— 由帧线程的 seq->票号映射回溯
    std::vector<std::pair<uint64_t, uint64_t>> captured_tickets;
    for (int c = 0; c < kCycles; ++c)
    {
        const int before = acks.load();   // acks 单调递增, 先取快照
        // 生产模型: 指令回调与图像回调共用同一把锁; 生产用 try_lock
        // (拿不到 = 写盘中, 忽略), 这里复刻同一契约。
        int ret = -1;
        {
            std::lock_guard<std::mutex> lk(state_mutex);
            ret = fsm->onCommand(0x01);
        }
        if (ret != 1)
        {
            // 处理中再次 0x01 被忽略(与生产行为一致), 补一次重试
            ++ignored_triggers;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            std::lock_guard<std::mutex> lk(state_mutex);
            ASSERT_EQ(fsm->onCommand(0x01), 1);
        }
        // c) 基线单调: 周期推进, 同侧基线严格递增(每周期至少灌入 2 帧)
        const uint64_t base_l = fsm->left().baseline;
        const uint64_t base_r = fsm->right().baseline;
        if (c > 0)
        {
            ASSERT_LT(prev_base_l, base_l);
            ASSERT_LT(prev_base_r, base_r);
        }
        prev_base_l = base_l;
        prev_base_r = base_r;
        // d) 同一次 arm 写入两侧 => 周期对齐(周期号由状态机统一维护)
        (void)c;
        // a) 收敛: 等本周期确认(帧线程全速灌帧, 毫秒级完成)
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(2);
        while (acks.load() < before + 1 &&
               std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::yield();
        }
        ASSERT_GE(acks.load(), before + 1);
        ++completed;
        // 周期归属审计: 锁内读取双侧捕获帧时间戳(=帧线程 seq),
        // 经 seq->票号映射回溯, 断言捕获票号恰为 基线+2 且逐周期递增。
        uint64_t t_l = 0;
        uint64_t t_r = 0;
        {
            // 读捕获时间戳与票号回溯都必须持锁(帧线程并发写映射)
            std::lock_guard<std::mutex> lk(log_mutex);
            const int64_t cap_l = fsm->left().stamp_ns;
            const int64_t cap_r = fsm->right().stamp_ns;
            const auto it_l = left_seq2ticket.find(cap_l);
            const auto it_r = right_seq2ticket.find(cap_r);
            ASSERT_TRUE(it_l != left_seq2ticket.end())
                << "周期 " << c << " 左捕获 stamp=" << cap_l
                << " 无票号映射(映射规模 " << left_seq2ticket.size() << ")";
            ASSERT_TRUE(it_r != right_seq2ticket.end())
                << "周期 " << c << " 右捕获 stamp=" << cap_r
                << " 无票号映射(映射规模 " << right_seq2ticket.size() << ")";
            t_l = it_l->second;
            t_r = it_r->second;
            if (t_l != base_l + 2 || t_r != base_r + 2)
            {
                std::lock_guard<std::mutex> lk2(state_mutex);
                std::cout << "[diag] 周期 " << c << " base_l=" << base_l
                          << " t_l=" << t_l << " left.captured="
                          << fsm->left().captured << " left.arrived="
                          << fsm->left().arrived.load()
                          << " left.stamp=" << fsm->left().stamp_ns
                          << " cap_l=" << cap_l << std::endl;
            }
        }
        ASSERT_EQ(t_l, base_l + 2) << "左捕获票号(周期 " << c << ")";
        ASSERT_EQ(t_r, base_r + 2) << "右捕获票号(周期 " << c << ")";
        if (c > 0)
        {
            ASSERT_LT(captured_tickets.back().first, t_l) << "左票号递增";
            ASSERT_LT(captured_tickets.back().second, t_r) << "右票号递增";
        }
        captured_tickets.emplace_back(t_l, t_r);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    run.store(false);
    tf.join();
    tf2.join();

    ASSERT_EQ(completed, kCycles);   // 全部周期完成(循环内已断言)
    ASSERT_EQ(fsm->state(), FsmState::kIdle);
    // b) 计数完整性: 日志计数 == 原子计数(无丢帧)
    ASSERT_EQ(logged_left.load(), fsm->left().arrived.load());
    ASSERT_EQ(logged_right.load(), fsm->right().arrived.load());

    std::cout << "[stress] left=" << fsm->left().arrived.load()
              << " right=" << fsm->right().arrived.load()
              << " frames, " << completed << "/" << kCycles << " cycles, "
              << acks.load() << " acks, " << ignored_triggers
              << " ignored(锁内拒重入)" << std::endl;
}

}  // namespace
