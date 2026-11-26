#include "lite3_recovery/controller.hpp"
#include "lite3_recovery/kinematics.hpp"
#include "lite3_recovery/profile_loader.hpp"
#include "lite3_recovery/reservation.hpp"
#include "lite3_recovery/standup.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using namespace lite3_recovery;

#define CHECK(expr) do { if (!(expr)) { std::cerr << __FILE__ << ":" << __LINE__ << ": CHECK failed: " #expr "\n"; std::exit(1); } } while(false)

static constexpr Commissioning kMockCommissioned{true, true, true, true};

static std::string profile_path() {
    const auto p = std::filesystem::path(__FILE__).parent_path().parent_path()
        / "config" / "left_roll_profile.json";
    return p.string();
}

static void test_profile_and_kinematics() {
    auto profile = load_profile_json(profile_path(), 1.0, true);
    CHECK(profile.segments.size() == 6);
    CHECK(valid_bounds(profile.bounds));
    CHECK(valid_trajectory(profile.start, profile.segments, profile.bounds));

    // Verify soft-gain loading also passes all checks
    auto soft = load_profile_json(profile_path(), 0.7, true);
    CHECK(soft.segments[1].target[0].kp < profile.segments[1].target[0].kp);

    // Check RPY -> Quaternion tilt accuracy
    const auto q_upright = rpy_deg_to_quat_xyzw(0.0, 0.0, 0.0);
    CHECK(unit(q_upright));
    CHECK(tilt(q_upright) < 0.1);

    const auto q_supine = rpy_deg_to_quat_xyzw(180.0, 0.0, 0.0);
    CHECK(unit(q_supine));
    CHECK(std::abs(tilt(q_supine) - 180.0) < 0.1);

    const auto q_side = rpy_deg_to_quat_xyzw(90.0, 0.0, 0.0);
    CHECK(std::abs(tilt(q_side) - 90.0) < 0.1);

    // Check FK height at standing vs prone
    OfficialStandUp standup;
    Axis q_stand{};
    const double hy_st = OfficialStandUp::hipy_by_height(0.30);
    const double kn_st = OfficialStandUp::knee_by_height(0.30);
    for (std::size_t leg = 0; leg < 4; ++leg) {
        q_stand[3*leg + 0] = 0.0;
        q_stand[3*leg + 1] = hy_st;
        q_stand[3*leg + 2] = kn_st;
    }
    const double h_stand = estimate_base_height_m(q_stand, q_upright);
    CHECK(h_stand > 0.27 && h_stand < 0.34);

    const double h_prone = estimate_base_height_m(standup.pre_pose(), q_upright);
    CHECK(h_prone > 0.08 && h_prone < 0.16);

    Feedback fb{};
    fb.position = standup.pre_pose();
    fb.attitude = q_upright;
    auto chk = assess_prone_support(fb, {0, 0, 0, 0}, standup.pre_pose());
    CHECK(chk.confirmed);

    fb.attitude = q_supine;
    chk = assess_prone_support(fb, {0, 0, 0, 0}, standup.pre_pose());
    CHECK(!chk.confirmed);
}

static void test_decoupled_detection_and_full_roll_stand() {
    auto profile = load_profile_json(profile_path(), 1.0, true);
    SequenceConfig cfg{};
    cfg.fall_tilt_deg = 65.0;
    cfg.supine_tilt_deg = 120.0;
    cfg.detect_dwell_ms = 300;
    cfg.stable_dwell_ms = 5000;

    Controller ctrl(profile, cfg);
    CHECK(ctrl.enable(77, kMockCommissioned, true));

    OfficialStandUp standup;
    Input in{};
    in.exclusive_authority = true;
    in.fall_allowed = true;
    // Start from realistic resting supine pose (slightly offset from profile.start)
    in.feedback.position = {
        -0.35, -1.70, 2.70,  0.38, -1.70, 2.70,
        -0.42, -1.68, 2.70,  0.42, -1.68, 2.70
    };
    in.feedback.attitude = rpy_deg_to_quat_xyzw(180.0, 0.0, 0.0);

    bool saw_early_detect = false;
    bool reached_upright = false;
    Joints last_sent{};

    auto sink = [&](Output out_type, const Joints& cmd) {
        CHECK(out_type == Output::motion);
        last_sent = cmd;
        return true;
    };

    for (Millis now = 100; now <= 13000; now += 5) {
        ++in.feedback.sequence;
        in.feedback.received_ms = now;

        // Track commanded pose smoothly once rolling/standing begins
        if (now > 5100) {
            for (std::size_t i = 0; i < joint_count; ++i) {
                in.feedback.velocity[i] = (last_sent[i].position - in.feedback.position[i]) / 0.005;
                in.feedback.position[i] = last_sent[i].position;
            }
        }
        // Simulate roll attitude reaching prone by end of rolling (5100 + 3900 = 9000ms)
        if (now >= 8500) {
            in.feedback.attitude = rpy_deg_to_quat_xyzw(2.0, 0.0, 0.0);
        }
        auto prone_chk = assess_prone_support(in.feedback, {0, 0, 0, 0}, standup.pre_pose());
        in.prone_confirmed = prone_chk.confirmed;
        in.expects_progress = standup.expects_progress(now);
        in.stand_complete = standup.complete(now);

        auto st = ctrl.advance(now, in, [&](Phase phase, Joints& out) {
            return standup.compute(now, in.feedback, 60.0, phase, out);
        });
        CHECK(st.stop == StopReason::none);
        if (st.phase == Phase::watching && st.fall_detected) {
            CHECK(st.fall_detected_ms == 400); // 100 + 300ms dwell (<= 2000ms)
            saw_early_detect = true;
        }
        if (st.phase != Phase::watching) {
            CHECK(ctrl.guard().tick(now, true, sink) == Output::motion);
        }
        if (st.phase == Phase::upright_hold) {
            reached_upright = true;
            break;
        }
    }
    CHECK(saw_early_detect);
    CHECK(reached_upright);
}

static void test_prone_shortcut() {
    auto profile = load_profile_json(profile_path(), 1.0, true);
    SequenceConfig cfg{};
    cfg.fall_tilt_deg = 65.0;
    cfg.supine_tilt_deg = 120.0;
    cfg.detect_dwell_ms = 300;
    cfg.stable_dwell_ms = 5000;
    cfg.allow_prone_shortcut = true;

    Controller ctrl(profile, cfg);
    CHECK(ctrl.enable(88, kMockCommissioned, true));

    OfficialStandUp standup;
    Input in{};
    in.exclusive_authority = true;
    in.fall_allowed = true;
    in.feedback.position = standup.pre_pose();
    // Fell flat/prone (tilt = 5 deg) with prone_confirmed=true
    in.feedback.attitude = rpy_deg_to_quat_xyzw(5.0, 0.0, 0.0);
    in.prone_confirmed = true;

    bool entered_rolling = false;
    bool reached_upright = false;
    Joints last_sent{};
    for (std::size_t i = 0; i < joint_count; ++i) last_sent[i].position = in.feedback.position[i];

    for (Millis now = 100; now <= 10000; now += 5) {
        ++in.feedback.sequence;
        in.feedback.received_ms = now;
        if (now > 5100) {
            for (std::size_t i = 0; i < joint_count; ++i) {
                in.feedback.velocity[i] = (last_sent[i].position - in.feedback.position[i]) / 0.005;
                in.feedback.position[i] = last_sent[i].position;
            }
        }
        in.expects_progress = standup.expects_progress(now);
        in.stand_complete = standup.complete(now);
        auto st = ctrl.advance(now, in, [&](Phase phase, Joints& out) {
            return standup.compute(now, in.feedback, 0.0, phase, out);
        });
        CHECK(st.stop == StopReason::none);
        if (st.phase == Phase::rolling) entered_rolling = true;
        if (st.phase != Phase::watching) {
            CHECK(ctrl.guard().tick(now, true, [&](Output t, const Joints& c) {
                CHECK(t == Output::motion);
                last_sent = c;
                return true;
            }) == Output::motion);
        }
        if (st.phase == Phase::upright_hold) {
            reached_upright = true;
            break;
        }
    }
    CHECK(!entered_rolling);
    CHECK(reached_upright);
}

static void test_multithreaded_realtime_stop_latency() {
    auto profile = load_profile_json(profile_path(), 1.0, true);
    SequenceConfig cfg{};
    cfg.stable_dwell_ms = 50; // Fast trigger for real-time thread test
    cfg.detect_dwell_ms = 10;

    Controller ctrl(profile, cfg);
    CHECK(ctrl.enable(99, kMockCommissioned, true));

    auto now_ms = []() -> Millis {
        using namespace std::chrono;
        return static_cast<Millis>(
            duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
    };

    std::atomic<bool> done{false};
    std::atomic<bool> motion_started{false};
    std::atomic<bool> stop_injected{false};
    std::atomic<Millis> stop_ts{0};
    std::atomic<int> motion_after_stop{0};
    std::atomic<int> damping_frames{0};

    // 1000Hz Guard transport thread
    std::thread tx_thread([&]() {
        while (!done.load()) {
            const Millis t = now_ms();
            ctrl.guard().tick(t, true, [&](Output out_type, const Joints& cmd) {
                if (out_type == Output::motion) {
                    motion_started.store(true);
                    if (stop_injected.load()) motion_after_stop.fetch_add(1);
                } else if (out_type == Output::damping) {
                    for (const auto& j : cmd) {
                        CHECK(j.kp == 0.0 && j.kd == 2.5 && j.position == 0.0 && j.velocity == 0.0);
                    }
                    damping_frames.fetch_add(1);
                }
                return true;
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // 200Hz Controller producer thread
    std::thread ctrl_thread([&]() {
        Input in{};
        in.exclusive_authority = true;
        in.fall_allowed = true;
        in.feedback.position = {
            -0.05, -1.36, 2.55,  0.05, -1.36, 2.55,
            -0.05, -1.36, 2.55,  0.05, -1.36, 2.55
        };
        in.feedback.attitude = rpy_deg_to_quat_xyzw(180.0, 0.0, 0.0);
        OfficialStandUp standup;
        Millis last_t = 0;
        while (!done.load()) {
            Millis t = now_ms();
            if (t <= last_t) t = last_t + 1;
            last_t = t;
            ++in.feedback.sequence;
            in.feedback.received_ms = t;
            ctrl.advance(t, in, [&](Phase phase, Joints& out) {
                return standup.compute(t, in.feedback, 60.0, phase, out);
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    // Wait for motion frames, then inject STOP asynchronously
    const Millis wait_deadline = now_ms() + 2000;
    while (!motion_started.load() && now_ms() < wait_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    CHECK(motion_started.load());

    const Millis t_stop = now_ms();
    stop_ts.store(t_stop);
    ctrl.guard().stop(StopReason::handset, t_stop);
    stop_injected.store(true);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    done.store(true);
    ctrl_thread.join();
    tx_thread.join();

    const auto ev = ctrl.guard().evidence();
    CHECK(ev.reason == StopReason::handset);
    CHECK(ev.damping_sent);
    CHECK(damping_frames.load() >= 10);
    CHECK(motion_after_stop.load() == 0);
    CHECK(ev.first_damping_ms >= ev.stop_ms);
    CHECK(ev.first_damping_ms - ev.stop_ms <= 20);
    std::cout << "Async STOP->damping latency: " << (ev.first_damping_ms - ev.stop_ms) << " ms\n";
}

int main() {
    test_profile_and_kinematics();
    test_decoupled_detection_and_full_roll_stand();
    test_prone_shortcut();
    test_multithreaded_realtime_stop_latency();
    std::cout << "All profile, kinematics, standup, and multithreaded transport mock tests passed\n";
    return 0;
}
