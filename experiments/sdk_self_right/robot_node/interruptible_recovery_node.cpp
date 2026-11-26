// Single-process interruptible SDK recovery node for DeepRobotics Lite3.
//
// Safety invariants:
//   1. Default mode is --mode dry-run (read-only Receiver, no Sender, no ControlGet).
//   2. Never calls Sender::RobotStateInit() (which returns all joints to zero).
//   3. Hardware feedback freshness comes strictly from Receiver::RegisterCallBack(0x0906)
//      with CLOCK_MONOTONIC timestamps, never from a republish timer.
//   4. Every UDP Sender::SendCmd() call executes inside lite3_recovery::Guard::tick()'s
//      synchronized sink (20 ms TTL, session/sequence checks, rate & effort bounds,
//      200 ms hardware feedback watchdog, pure-damping fallback).
//   5. All STOP sources (Retroid gamepad, deeprcs Soft Emergency log, ROS 2
//      /EMERGENCY_STOP service, local terminal CANCEL / SIGINT / SIGTERM) latch
//      Guard::stop() in-process under the same mutex as SendCmd().

#include "rclcpp/rclcpp.hpp"
#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/imu_data.hpp"
#include "drdds/msg/gamepad_data.hpp"
#include "drdds/srv/std_srv_int32.hpp"

#include "receiver.h"
#include "sender.h"
#include "robot_types.h"
#include "retroid_gamepad.h"

#include "lite3_recovery/controller.hpp"
#include "lite3_recovery/kinematics.hpp"
#include "lite3_recovery/profile_loader.hpp"
#include "lite3_recovery/reservation.hpp"
#include "lite3_recovery/standup.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;
using namespace lite3_recovery;

namespace {

std::atomic<bool> g_signal_stop{false};

void signal_handler(int) {
    g_signal_stop.store(true);
}

Millis steady_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<Millis>(ts.tv_sec) * 1000ULL + static_cast<Millis>(ts.tv_nsec / 1000000ULL);
}

void sleep_until_ns(uint64_t target_ns) {
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(target_ns / 1000000000ULL);
    ts.tv_nsec = static_cast<long>(target_ns % 1000000000ULL);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
}

uint64_t steady_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

const char* phase_name(Phase p) {
    switch (p) {
        case Phase::disabled: return "disabled";
        case Phase::watching: return "watching";
        case Phase::rolling: return "rolling";
        case Phase::prone_check: return "prone_check";
        case Phase::standing: return "standing";
        case Phase::upright_hold: return "upright_hold";
        case Phase::stopped: return "stopped";
    }
    return "unknown";
}

const char* stop_reason_name(StopReason r) {
    switch (r) {
        case StopReason::none: return "none";
        case StopReason::handset: return "handset";
        case StopReason::cancel: return "cancel";
        case StopReason::stall: return "stall";
        case StopReason::feedback: return "feedback";
        case StopReason::fault: return "fault";
        case StopReason::timeout: return "timeout";
        case StopReason::command_expired: return "command_expired";
        case StopReason::transport: return "transport";
        case StopReason::clock: return "clock";
        case StopReason::ownership: return "ownership";
    }
    return "unknown";
}

bool udp_port_available(int port) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    const bool ok = (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    ::close(fd);
    return ok;
}

struct Options {
    std::string mode{"dry-run"}; // dry-run | stop-test | recover
    std::string stand_mode{"native-handoff"}; // native-handoff | sdk
    std::string robot_ip{"192.168.1.120"};
    int robot_port{43893};
    int gamepad_port{12121};
    int duration_s{120};
    Millis stable_wait_ms{2000};
    double kp_scale{1.0};
    double damping_hold_s{0.25};
    bool damping_hold_explicit{false};
    double stand_hold_s{2.0};
    std::string profile_path;
    std::string audit_path;
    std::string lock_dir;
    std::string deeprcs_log_root{"/home/ysc/jy_exe/log"};
    bool confirm_sling{false};
    bool allow_prone_shortcut{false};
    bool continuous{false};
    uint16_t retroid_stop_mask{0xC000}; // Both joystick buttons by default
    bool invert_rpy{false};
};

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "  --mode dry-run|stop-test|recover   (default: dry-run)\n"
              << "  --stand-mode native-handoff|sdk    (default: native-handoff; stands & returns handset control)\n"
              << "  --continuous                       (keep observing until CANCEL/SIGTERM/shutdown)\n"
              << "  --robot-ip <ip>                    (default: 192.168.1.120)\n"
              << "  --robot-port <port>                (default: 43893)\n"
              << "  --gamepad-port <port>              (default: 12121)\n"
              << "  --duration <sec>                   (default: 120, range 1..86400; ignored with --continuous)\n"
              << "  --stable-wait-ms <ms>              (default: 2000, stable fallen wait before recovery)\n"
              << "  --kp-scale <0.2..1.0>              (default: 1.0)\n"
              << "  --profile <json>                   (required for recover)\n"
              << "  --audit <new.jsonl>                (required for stop-test/recover)\n"
              << "  --lock-dir <dir>                   (default: $HOME/.lite3-recovery-attempts)\n"
              << "  --deeprcs-log-root <dir>           (default: /home/ysc/jy_exe/log)\n"
              << "  --allow-prone-shortcut             (allow <120 deg prone shortcut)\n"
              << "  --confirm-sling                    (required for stop-test/recover)\n";
}

bool parse_args(int argc, char** argv, Options& opt) {
    const char* home = std::getenv("HOME");
    opt.lock_dir = home ? (std::string(home) + "/.lite3-recovery-attempts") : "/tmp/lite3-recovery-attempts";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next_val = [&](const std::string& flag) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Missing value for " + flag);
            return argv[++i];
        };
        if (a == "--help" || a == "-h") {
            print_usage(argv[0]);
            return false;
        } else if (a == "--mode") {
            opt.mode = next_val(a);
        } else if (a == "--stand-mode") {
            opt.stand_mode = next_val(a);
        } else if (a == "--continuous") {
            opt.continuous = true;
        } else if (a == "--robot-ip") {
            opt.robot_ip = next_val(a);
        } else if (a == "--robot-port") {
            opt.robot_port = std::stoi(next_val(a));
        } else if (a == "--gamepad-port") {
            opt.gamepad_port = std::stoi(next_val(a));
        } else if (a == "--duration") {
            opt.duration_s = std::stoi(next_val(a));
        } else if (a == "--stable-wait-ms") {
            opt.stable_wait_ms = static_cast<Millis>(std::stoull(next_val(a)));
        } else if (a == "--kp-scale") {
            opt.kp_scale = std::stod(next_val(a));
        } else if (a == "--damping-hold-s") {
            opt.damping_hold_s = std::stod(next_val(a));
            opt.damping_hold_explicit = true;
        } else if (a == "--profile") {
            opt.profile_path = next_val(a);
        } else if (a == "--audit") {
            opt.audit_path = next_val(a);
        } else if (a == "--lock-dir") {
            opt.lock_dir = next_val(a);
        } else if (a == "--deeprcs-log-root") {
            opt.deeprcs_log_root = next_val(a);
        } else if (a == "--allow-prone-shortcut") {
            opt.allow_prone_shortcut = true;
        } else if (a == "--confirm-sling") {
            opt.confirm_sling = true;
        } else {
            throw std::runtime_error("Unknown option: " + a);
        }
    }
    if (opt.mode != "dry-run" && opt.mode != "stop-test" && opt.mode != "recover")
        throw std::runtime_error("Invalid --mode: " + opt.mode);
    if (opt.stand_mode != "native-handoff" && opt.stand_mode != "sdk")
        throw std::runtime_error("Invalid --stand-mode: " + opt.stand_mode);
    if (opt.duration_s < 1 || opt.duration_s > 86400)
        throw std::runtime_error("--duration must be in [1, 86400]");
    if (opt.stable_wait_ms < 300 || opt.stable_wait_ms > 30000)
        throw std::runtime_error("--stable-wait-ms must be in [300, 30000]");
    if (!within(opt.kp_scale, 0.2, 1.0))
        throw std::runtime_error("--kp-scale must be in [0.2, 1.0]");
    if (opt.mode == "stop-test" && !opt.damping_hold_explicit) {
        opt.damping_hold_s = 3.0;
    }
    if (opt.mode != "dry-run") {
        if (!opt.confirm_sling)
            throw std::runtime_error("Active modes require --confirm-sling");
        if (opt.audit_path.empty())
            throw std::runtime_error("Active modes require a new --audit <file.jsonl> path");
        if (opt.mode == "recover" && opt.profile_path.empty())
            throw std::runtime_error("--mode recover requires --profile <left_roll_profile.json>");
    }
    return true;
}

class AuditLog {
public:
    void open_new(const std::string& path) {
        if (path.empty()) return;
        const auto parent = fs::path(path).parent_path();
        if (!parent.empty()) fs::create_directories(parent);
        int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
        if (fd < 0) {
            throw std::runtime_error("Refusing to overwrite or cannot create audit file: " + path
                                     + " (" + std::strerror(errno) + ")");
        }
        ::close(fd);
        ofs_.open(path, std::ios::out | std::ios::app);
        if (!ofs_) throw std::runtime_error("Cannot open audit stream: " + path);
    }

    void write_line(const std::string& json_line, bool echo_stdout = true) {
        std::lock_guard<std::mutex> lock(mu_);
        if (ofs_.is_open()) {
            ofs_ << json_line << '\n';
            ofs_.flush();
        }
        if (echo_stdout) {
            std::cout << json_line << std::endl;
        }
    }

private:
    std::mutex mu_;
    std::ofstream ofs_;
};

std::string json_axis(const Axis& a) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(4) << '[';
    for (std::size_t i = 0; i < joint_count; ++i) {
        if (i) os << ',';
        os << a[i];
    }
    os << ']';
    return os.str();
}

struct HwSnapshot {
    uint64_t seq{0};
    Millis mono_ms{0};
    uint32_t tick{0};
    Axis pos{};
    Axis vel{};
    Axis tau{};
    std::array<double, 3> rpy_deg{};
    std::array<double, 3> omega{};
    std::array<double, 3> acc{};
    std::array<double, 4> contact_z{};
};

// Tails the newest deeprcs.*.log file starting strictly from EOF at startup.
class DeepRcsLogWatcher {
public:
    explicit DeepRcsLogWatcher(std::string root) : root_(std::move(root)) {}

    bool init_at_eof(std::string& found_path) {
        auto latest = find_latest();
        if (latest.empty()) return false;
        path_ = latest;
        found_path = path_;
        struct stat st{};
        if (::stat(path_.c_str(), &st) == 0) {
            offset_ = static_cast<uint64_t>(st.st_size);
        }
        return true;
    }

    std::vector<std::string> poll_new_lines() {
        std::vector<std::string> out;
        if (path_.empty()) return out;
        std::ifstream ifs(path_, std::ios::binary);
        if (!ifs) return out;
        ifs.seekg(0, std::ios::end);
        const auto sz = static_cast<uint64_t>(ifs.tellg());
        if (sz <= offset_) return out;
        ifs.seekg(static_cast<std::streamoff>(offset_), std::ios::beg);
        const uint64_t to_read = std::min<uint64_t>(sz - offset_, 65536);
        std::string chunk(static_cast<std::size_t>(to_read), '\0');
        ifs.read(chunk.data(), static_cast<std::streamsize>(to_read));
        const auto got = static_cast<uint64_t>(ifs.gcount());
        offset_ += got;
        chunk.resize(static_cast<std::size_t>(got));
        partial_ += chunk;
        std::size_t pos = 0;
        while (true) {
            auto nl = partial_.find('\n', pos);
            if (nl == std::string::npos) break;
            out.push_back(partial_.substr(pos, nl - pos));
            pos = nl + 1;
        }
        partial_.erase(0, pos);
        return out;
    }

private:
    std::string find_latest() const {
        if (!fs::exists(root_)) return {};
        std::string best;
        fs::file_time_type best_time{};
        bool has_best = false;
        std::error_code ec;
        for (const auto& entry : fs::recursive_directory_iterator(root_, ec)) {
            if (!entry.is_regular_file(ec)) continue;
            const auto name = entry.path().filename().string();
            if (name.rfind("deeprcs.", 0) == 0 && name.size() > 4
                && name.substr(name.size() - 4) == ".log") {
                const auto mt = entry.last_write_time(ec);
                if (!has_best || mt > best_time) {
                    has_best = true;
                    best_time = mt;
                    best = entry.path().string();
                }
            }
        }
        return best;
    }

    std::string root_;
    std::string path_;
    uint64_t offset_{0};
    std::string partial_;
};

// Mirrors a validated left-roll Profile into a right-roll Profile by swapping
// FL(0..2) <-> FR(3..5), HL(6..8) <-> HR(9..11) and negating HipX positions.
Profile mirror_profile_right(const Profile& left) {
    auto mirror_joints = [](const Joints& in) {
        Joints out{};
        for (const auto [l, r] : std::array<std::pair<std::size_t, std::size_t>, 2>{{{0, 3}, {6, 9}}}) {
            out[l + 0] = in[r + 0]; out[l + 0].position = -in[r + 0].position; out[l + 0].velocity = -in[r + 0].velocity;
            out[l + 1] = in[r + 1];
            out[l + 2] = in[r + 2];
            out[r + 0] = in[l + 0]; out[r + 0].position = -in[l + 0].position; out[r + 0].velocity = -in[l + 0].velocity;
            out[r + 1] = in[l + 1];
            out[r + 2] = in[l + 2];
        }
        return out;
    };
    Profile right = left;
    right.start = mirror_joints(left.start);
    for (auto& seg : right.segments) {
        seg.target = mirror_joints(seg.target);
    }
    return right;
}

class InterruptibleRecoveryApp {
public:
    explicit InterruptibleRecoveryApp(Options opt) : opt_(std::move(opt)) {}

    int run() {
        const Millis start_ms = steady_ms();
        const uint64_t session_id = static_cast<uint64_t>(
            std::chrono::system_clock::now().time_since_epoch().count());

        audit_.open_new(opt_.audit_path);

        if (!udp_port_available(43897)) {
            throw std::runtime_error(
                "UDP port 43897 is already in use (stop native_state receiver or lite3_transfer first)");
        }

        // Initialize optional ROS 2 node for /EMERGENCY_STOP service and read-only telemetry topics.
        ros_node_ = std::make_shared<rclcpp::Node>("lite3_interruptible_recovery");
        joints_pub_ = ros_node_->create_publisher<drdds::msg::JointsData>("/JOINTS_DATA", 10);
        imu_pub_ = ros_node_->create_publisher<drdds::msg::ImuData>("/IMU_DATA", 10);
        gamepad_pub_ = ros_node_->create_publisher<drdds::msg::GamepadData>("/GAMEPAD_DATA", 10);
        // ROS 2 /EMERGENCY_STOP aborts active recovery (or exits single-shot mode).
        estop_srv_ = ros_node_->create_service<drdds::srv::StdSrvInt32>(
            "/EMERGENCY_STOP",
            [this](const std::shared_ptr<drdds::srv::StdSrvInt32::Request>,
                   std::shared_ptr<drdds::srv::StdSrvInt32::Response> resp) {
                trigger_stop(StopReason::cancel, "ros2_emergency_stop_service", !opt_.continuous);
                resp->result = 0;
            });

        // Start Retroid gamepad listener on UDP gamepad_port (best effort if port is free).
        if (udp_port_available(opt_.gamepad_port)) {
            gamepad_ = std::make_shared<RetroidGamepad>(opt_.gamepad_port);
            gamepad_->StartDataThread();
        } else {
            log_event("warn", "\"detail\":\"gamepad_port_busy_skipping_retroid\"");
        }

        // Initialize deeprcs live log watcher at EOF so historical lines never trigger STOP.
        deeprcs_watcher_ = std::make_unique<DeepRcsLogWatcher>(opt_.deeprcs_log_root);
        std::string watched_log;
        const bool log_ok = deeprcs_watcher_->init_at_eof(watched_log);

        {
            std::ostringstream ex;
            ex << "\"mode\":\"" << opt_.mode << "\""
               << ",\"stand_mode\":\"" << opt_.stand_mode << "\""
               << ",\"continuous\":" << (opt_.continuous ? "true" : "false")
               << ",\"session\":" << session_id
               << ",\"robot_ip\":\"" << opt_.robot_ip << "\""
               << ",\"robot_port\":" << opt_.robot_port
               << ",\"kp_scale\":" << opt_.kp_scale
               << ",\"deeprcs_log\":\"" << (log_ok ? watched_log : "") << "\"";
            log_event("startup", ex.str());
        }

        // Load profile & prepare controller / guard BEFORE touching hardware Receiver/Sender.
        Profile profile{};
        if (opt_.mode == "recover") {
            profile = load_profile_json(opt_.profile_path, opt_.kp_scale, true);
        } else if (opt_.mode == "stop-test") {
            standalone_guard_ = std::make_unique<Guard>();
            active_guard_ = standalone_guard_.get();
        }

        // Create read-only Receiver and attach genuine 0x0906 frame callback.
        receiver_ = std::make_unique<Receiver>();
        receiver_->RegisterCallBack([this](int code) { on_hw_frame(code); });
        receiver_->StartWork();

        // Wait up to 3 s for live 0x0906 frames before proceeding.
        const Millis wait_deadline = steady_ms() + 3000;
        while (steady_ms() < wait_deadline && !g_signal_stop.load()) {
            if (get_hw_snapshot().seq >= 5) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        auto snap0 = get_hw_snapshot();
        if (snap0.seq < 5) {
            throw std::runtime_error("No 0x0906 hardware frames received on UDP 43897 within 3 seconds");
        }

        // Start ROS 2 spin thread, gamepad watcher, deeprcs log watcher, and stdin CANCEL watcher.
        ros_thread_ = std::thread([this]() {
            rclcpp::executors::SingleThreadedExecutor exec;
            exec.add_node(ros_node_);
            while (!shutdown_.load() && rclcpp::ok()) {
                exec.spin_some(std::chrono::milliseconds(5));
            }
        });
        watcher_thread_ = std::thread([this]() { stop_watcher_loop(); });
        stdin_thread_ = std::thread([this]() { stdin_cancel_loop(); });

        int rc = 0;
        if (opt_.mode == "dry-run") {
            rc = run_dry_run(start_ms);
        } else if (opt_.mode == "stop-test") {
            rc = run_stop_test(session_id, start_ms);
        } else {
            rc = run_recover(session_id, profile, start_ms);
        }

        shutdown_.store(true);
        if (tx_thread_.joinable()) tx_thread_.join();
        if (watcher_thread_.joinable()) watcher_thread_.join();
        if (stdin_thread_.joinable()) stdin_thread_.join();
        if (ros_thread_.joinable()) ros_thread_.join();

        if (sender_ && sdk_authority_.load()) {
            release_sdk_fast();
        }
        // Vendor Receiver and RetroidGamepad run detached blocking recvfrom threads;
        // avoid destroying their mutexes while process is exiting cleanly.
        if (receiver_) {
            receiver_->RegisterCallBack(nullptr);
            (void)receiver_.release();
        }
        if (gamepad_) {
            gamepad_->StopDataThread();
            // Intentionally keep alive until process exit so detached thread never locks destroyed mutex.
            static std::shared_ptr<RetroidGamepad> leaked_gamepad;
            leaked_gamepad = std::move(gamepad_);
        }
        return rc;
    }

private:
    void on_hw_frame(int code) {
        if (code != 0x0906 || !receiver_) return;
        const Millis now = steady_ms();
        const RobotData& rd = receiver_->GetState();

        HwSnapshot s{};
        {
            std::lock_guard<std::mutex> lock(hw_mu_);
            s.seq = ++hw_seq_;
            s.mono_ms = now;
            s.tick = rd.tick;
            for (std::size_t i = 0; i < joint_count; ++i) {
                s.pos[i] = rd.joint_data.joint_data[i].position;
                s.vel[i] = rd.joint_data.joint_data[i].velocity;
                s.tau[i] = rd.joint_data.joint_data[i].torque;
            }
            s.rpy_deg = {rd.imu.angle_roll, rd.imu.angle_pitch, rd.imu.angle_yaw};
            s.omega = {rd.imu.angular_velocity_roll, rd.imu.angular_velocity_pitch, rd.imu.angular_velocity_yaw};
            s.acc = {rd.imu.acc_x, rd.imu.acc_y, rd.imu.acc_z};
            for (std::size_t leg = 0; leg < 4; ++leg) {
                s.contact_z[leg] = rd.contact_force.leg_force[3 * leg + 2];
            }
            latest_hw_ = s;
        }
    }

    HwSnapshot get_hw_snapshot() const {
        std::lock_guard<std::mutex> lock(hw_mu_);
        return latest_hw_;
    }

    void trigger_stop(StopReason reason, const char* source, bool exit_daemon = false) {
        if (exit_daemon) {
            cancel_requested_.store(true);
        }
        // In continuous mode while passively watching (no active recovery),
        // an operator handset STOP should reset the fall timer rather than killing the service.
        if (opt_.continuous && !exit_daemon && !recovery_active_.load()) {
            watching_estop_epoch_.fetch_add(1);
            std::ostringstream ex;
            ex << "\"reason\":\"" << stop_reason_name(reason) << "\""
               << ",\"source\":\"" << source << "\"";
            log_event("stop_while_watching_ignored", ex.str());
            return;
        }
        Millis now = 0;
        {
            std::lock_guard<std::mutex> glock(guard_call_mu_);
            now = steady_ms();
            if (active_guard_) {
                active_guard_->stop(reason, now);
            }
        }
        bool expected = false;
        if (stop_latched_.compare_exchange_strong(expected, true)) {
            stop_reason_.store(static_cast<int>(reason));
            stop_ms_.store(now);
            std::ostringstream ex;
            ex << "\"reason\":\"" << stop_reason_name(reason) << "\""
               << ",\"source\":\"" << source << "\"";
            log_event("stop_triggered", ex.str());
        }
    }

    void stop_watcher_loop() {
        uint16_t last_buttons = 0;
        while (!shutdown_.load()) {
            if (g_signal_stop.load()) {
                trigger_stop(StopReason::cancel, "posix_signal", true);
            }
            if (gamepad_) {
                RetroidKeys keys = gamepad_->GetKeys();
                last_gamepad_buttons_.store(keys.value);
                if ((keys.value & opt_.retroid_stop_mask) == opt_.retroid_stop_mask) {
                    trigger_stop(StopReason::handset, "retroid_gamepad_stop_mask", false);
                }
                if (keys.value != last_buttons) {
                    last_buttons = keys.value;
                    std::ostringstream ex;
                    ex << "\"buttons\":" << keys.value;
                    log_event("gamepad_buttons_changed", ex.str());
                }
            }
            if (deeprcs_watcher_) {
                for (const auto& line : deeprcs_watcher_->poll_new_lines()) {
                    if (line.find("Soft Emergency Pressed") != std::string::npos) {
                        trigger_stop(StopReason::handset, "deeprcs_soft_emergency", false);
                    } else if (line.find("[FATAL") != std::string::npos) {
                        trigger_stop(StopReason::fault, "deeprcs_fatal", false);
                    } else if (line.find("Robot is waitting to stand up") != std::string::npos) {
                        factory_waiting_standup_.store(true);
                    } else if (line.find("Standing up, please wait") != std::string::npos) {
                        factory_standing_started_.store(true);
                    } else if (line.find("Switching to force control mode") != std::string::npos
                               || line.find("Stand up action has finished") != std::string::npos) {
                        factory_force_control_.store(true);
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void stdin_cancel_loop() {
        if (!::isatty(STDIN_FILENO)) return;
        std::string line;
        while (!shutdown_.load()) {
            pollfd pfd{};
            pfd.fd = STDIN_FILENO;
            pfd.events = POLLIN;
            int r = ::poll(&pfd, 1, 50);
            if (r > 0 && (pfd.revents & POLLIN)) {
                if (!std::getline(std::cin, line)) break;
                if (line == "CANCEL" || line == "cancel" || line == "STOP" || line == "stop" || line == "q") {
                    trigger_stop(StopReason::cancel, "terminal_cancel", true);
                }
            }
        }
    }

    void publish_ros_telemetry(const HwSnapshot& s) {
        if (!ros_node_) return;
        const auto stamp = ros_node_->now();
        drdds::msg::JointsData jmsg;
        jmsg.header.stamp = stamp;
        jmsg.header.frame_id = static_cast<uint32_t>(s.seq);
        for (std::size_t i = 0; i < joint_count; ++i) {
            jmsg.data.joints_data[i].status_word = 1;
            jmsg.data.joints_data[i].position = static_cast<float>(s.pos[i]);
            jmsg.data.joints_data[i].velocity = static_cast<float>(s.vel[i]);
            jmsg.data.joints_data[i].torque = static_cast<float>(s.tau[i]);
        }
        joints_pub_->publish(jmsg);

        drdds::msg::ImuData imsg;
        imsg.header.stamp = stamp;
        imsg.data.roll = static_cast<float>(s.rpy_deg[0]);
        imsg.data.pitch = static_cast<float>(s.rpy_deg[1]);
        imsg.data.yaw = static_cast<float>(s.rpy_deg[2]);
        imsg.data.omega_x = static_cast<float>(s.omega[0]);
        imsg.data.omega_y = static_cast<float>(s.omega[1]);
        imsg.data.omega_z = static_cast<float>(s.omega[2]);
        imsg.data.acc_x = static_cast<float>(s.acc[0]);
        imsg.data.acc_y = static_cast<float>(s.acc[1]);
        imsg.data.acc_z = static_cast<float>(s.acc[2]);
        imu_pub_->publish(imsg);
    }

    void start_guard_tx_thread() {
        tx_thread_ = std::thread([this]() {
            uint64_t next_ns = steady_ns();
            while (!shutdown_.load()) {
                next_ns += 1000000ULL; // 1000 Hz
                if (sdk_authority_.load() && sender_) {
                    Millis now = 0;
                    Output out = Output::none;
                    Evidence ev{};
                    bool should_log_motion = false;
                    bool should_log_damping = false;
                    {
                        std::lock_guard<std::mutex> glock(guard_call_mu_);
                        if (sdk_authority_.load() && active_guard_) {
                            now = steady_ms();
                            const bool owns = sdk_authority_.load();
                            out = active_guard_->tick(
                                now, owns,
                                [&](Output, const Joints& cmd) -> bool {
                                    RobotCmd rcmd{};
                                    for (std::size_t i = 0; i < joint_count; ++i) {
                                        rcmd.joint_cmd[i].position = static_cast<float>(cmd[i].position);
                                        rcmd.joint_cmd[i].velocity = static_cast<float>(cmd[i].velocity);
                                        rcmd.joint_cmd[i].kp = static_cast<float>(cmd[i].kp);
                                        rcmd.joint_cmd[i].kd = static_cast<float>(cmd[i].kd);
                                        rcmd.joint_cmd[i].torque = 0.0f; // Feedforward strictly zero
                                    }
                                    sender_->SendCmd(rcmd);
                                    return true;
                                });
                            if (out == Output::motion) {
                                tx_motion_count_.fetch_add(1);
                                bool exp = false;
                                if (logged_first_motion_.compare_exchange_strong(exp, true)) {
                                    should_log_motion = true;
                                }
                            } else if (out == Output::damping) {
                                tx_damping_count_.fetch_add(1);
                                bool exp = false;
                                if (logged_first_damping_.compare_exchange_strong(exp, true)) {
                                    ev = active_guard_->evidence();
                                    should_log_damping = true;
                                }
                            }
                        }
                    }
                    if (should_log_motion) {
                        log_event("first_motion_sent", "\"tx_ms\":" + std::to_string(now));
                    }
                    if (should_log_damping) {
                        const Millis delay = (ev.first_damping_ms >= ev.stop_ms)
                            ? (ev.first_damping_ms - ev.stop_ms) : 0;
                        std::ostringstream ex;
                        ex << "\"reason\":\"" << stop_reason_name(ev.reason) << "\""
                           << ",\"stop_ms\":" << ev.stop_ms
                           << ",\"last_motion_ms\":" << ev.last_motion_ms
                           << ",\"first_damping_ms\":" << ev.first_damping_ms
                           << ",\"damping_delay_ms\":" << delay;
                        log_event("first_damping_sent", ex.str());
                    }
                }
                sleep_until_ns(next_ns);
            }
        });
    }

    int run_dry_run(Millis start_ms) {
        const Millis deadline = start_ms + static_cast<Millis>(opt_.duration_s) * 1000ULL;
        OfficialStandUp standup;
        uint64_t prev_seq = get_hw_snapshot().seq;
        Millis prev_report = steady_ms();

        while (!shutdown_.load() && !stop_latched_.load() && steady_ms() < deadline) {
            const auto s = get_hw_snapshot();
            publish_ros_telemetry(s);
            const Millis now = steady_ms();
            if (now - prev_report >= 500) {
                const double dt_s = static_cast<double>(now - prev_report) / 1000.0;
                const double hz = static_cast<double>(s.seq - prev_seq) / std::max(0.001, dt_s);
                prev_seq = s.seq;
                prev_report = now;
                const auto q = rpy_deg_to_quat_xyzw(s.rpy_deg[0], s.rpy_deg[1], s.rpy_deg[2]);
                Feedback fb{s.seq, s.mono_ms, s.pos, s.vel, q};
                const auto prone = assess_prone_support(fb, s.contact_z, standup.pre_pose());
                std::ostringstream ex;
                ex << std::fixed << std::setprecision(2)
                   << "\"rx_hz\":" << hz
                   << ",\"hw_age_ms\":" << (now >= s.mono_ms ? (now - s.mono_ms) : 0)
                   << ",\"tick\":" << s.tick
                   << ",\"rpy_deg\":[" << s.rpy_deg[0] << "," << s.rpy_deg[1] << "," << s.rpy_deg[2] << "]"
                   << ",\"tilt_deg\":" << tilt(q)
                   << ",\"est_height_m\":" << std::setprecision(3) << prone.est_height_m
                   << ",\"prone_confirmed\":" << (prone.confirmed ? "true" : "false")
                   << ",\"contact_z\":[" << std::setprecision(1)
                   << s.contact_z[0] << "," << s.contact_z[1] << ","
                   << s.contact_z[2] << "," << s.contact_z[3] << "]"
                   << ",\"gamepad_buttons\":" << last_gamepad_buttons_.load()
                   << ",\"pos\":" << json_axis(s.pos);
                log_event("dry_run_telemetry", ex.str());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        log_event("dry_run_complete", "\"elapsed_ms\":" + std::to_string(steady_ms() - start_ms));
        return 0;
    }

    int run_stop_test(uint64_t session_id, Millis start_ms) {
        fs::create_directories(opt_.lock_dir);
        ::chmod(opt_.lock_dir.c_str(), 0700);
        if (!reserve_trial(opt_.lock_dir, session_id)) {
            throw std::runtime_error(
                "Durable lock exists in " + opt_.lock_dir + "/unfinished-sdk-recovery; review & archive first");
        }
        log_event("trial_reserved", "\"lock_dir\":\"" + opt_.lock_dir + "\"");

        const auto init = get_hw_snapshot();
        // Tight safety bounds centered on current measured posture: only FL_Knee
        // gently moves by +-0.10 rad at low kp=25 so operator can verify SDK
        // authority and STOP->damping latency safely on the sling.
        Bounds bounds{};
        Axis damping{};
        for (std::size_t i = 0; i < joint_count; ++i) {
            const double span = (i == 2) ? 0.25 : 0.12;
            bounds.min_position[i] = init.pos[i] - span;
            bounds.max_position[i] = init.pos[i] + span;
            bounds.max_speed[i] = 3.0;
            bounds.max_kp[i] = 40.0;
            bounds.max_kd[i] = 5.0;
            bounds.max_effort[i] = 20.0;
            bounds.max_acceleration[i] = 30.0;
            bounds.max_gain_rate[i] = 200.0;
            damping[i] = 2.5;
        }
        Commissioning checks{true, true, true, true};
        {
            std::lock_guard<std::mutex> glock(guard_call_mu_);
            const Millis arm_now = steady_ms();
            if (!standalone_guard_->arm(session_id, arm_now, checks, true, bounds, damping)) {
                throw std::runtime_error("Failed to arm Guard for stop-test");
            }
            standalone_guard_->feedback(init.seq, init.mono_ms, arm_now, init.pos, init.vel);
        }
        guard_armed_.store(true);

        sender_ = std::make_unique<Sender>(opt_.robot_ip, static_cast<uint16_t>(opt_.robot_port));
        // NOTE: Never call sender_->RobotStateInit()!
        sender_->ControlGet(SDK);
        sdk_authority_.store(true);
        log_event("stop_test_armed", "\"joint\":\"FL_Knee\",\"amp_rad\":0.16,\"kp\":25.0");

        start_guard_tx_thread();

        const Millis deadline = start_ms + static_cast<Millis>(opt_.duration_s) * 1000ULL;
        const Millis motion_t0 = steady_ms();
        uint64_t cmd_seq = 0;
        uint64_t last_fb_seq = init.seq;
        Millis stop_seen_ms = 0;
        Millis settled_since = 0;
        Millis physical_settled_ms = 0;
        double last_max_vel = 0.0;
        uint64_t next_ns = steady_ns();

        while (!shutdown_.load() && steady_ms() < deadline) {
            next_ns += 5000000ULL; // 200 Hz
            const auto s = get_hw_snapshot();
            if (cmd_seq % 4 == 0) publish_ros_telemetry(s);

            double max_v = 0.0;
            for (double v : s.vel) max_v = std::max(max_v, std::abs(v));
            last_max_vel = max_v;

            const auto ev = standalone_guard_->evidence();
            if (ev.reason != StopReason::none) {
                const Millis now = steady_ms();
                if (stop_seen_ms == 0) stop_seen_ms = now;
                if (max_v < 0.15) {
                    if (settled_since == 0) settled_since = now;
                    if (physical_settled_ms == 0 && now - settled_since >= 300) {
                        physical_settled_ms = now;
                        log_event("physical_settled",
                                  "\"settled_ms\":" + std::to_string(now)
                                  + ",\"since_stop_ms\":" + std::to_string(now - ev.stop_ms));
                    }
                } else {
                    settled_since = 0;
                }
                if (now - stop_seen_ms >= static_cast<Millis>(opt_.damping_hold_s * 1000.0)) {
                    break;
                }
            } else {
                std::lock_guard<std::mutex> glock(guard_call_mu_);
                const Millis now = steady_ms();
                if (s.seq > last_fb_seq) {
                    standalone_guard_->feedback(s.seq, s.mono_ms, now, s.pos, s.vel);
                    last_fb_seq = s.seq;
                }
                const double t_s = static_cast<double>(now - motion_t0) / 1000.0;
                const double ramp = std::min(1.0, t_s / 0.6);
                const double target_kp = 25.0 * opt_.kp_scale * ramp;
                const double wave_t = std::max(0.0, t_s - 0.6);
                constexpr double kOmega = 2.0 * 3.14159265358979323846 * 0.5; // 0.5 Hz
                // Move strictly away from the closer mechanical limit ([0.55, 2.75]),
                // starting smoothly with zero velocity via (1 - cos(w*t))/2.
                const double dir = (init.pos[2] > 1.6) ? -1.0 : 1.0;
                const double raw_delta = dir * 0.16 * 0.5 * (1.0 - std::cos(kOmega * wave_t));
                const double knee_target = std::clamp(init.pos[2] + raw_delta, 0.55, 2.75);
                const double dvel = (t_s > 0.6 && knee_target > 0.55 && knee_target < 2.75)
                    ? (dir * 0.16 * 0.5 * kOmega * std::sin(kOmega * wave_t)) : 0.0;

                Frame frame{};
                frame.session = session_id;
                frame.sequence = ++cmd_seq;
                frame.source_ms = now;
                for (std::size_t i = 0; i < joint_count; ++i) {
                    frame.joints[i] = {
                        (i == 2 ? knee_target : init.pos[i]),
                        (i == 2 ? dvel : 0.0),
                        target_kp,
                        2.5,
                        0.0
                    };
                }
                standalone_guard_->accept(frame, now);
            }
            sleep_until_ns(next_ns);
        }

        const auto ev = standalone_guard_->evidence();
        std::ostringstream sum;
        sum << std::fixed << std::setprecision(4)
            << "\"stop_reason\":\"" << stop_reason_name(ev.reason) << "\""
            << ",\"stop_ms\":" << ev.stop_ms
            << ",\"last_motion_ms\":" << ev.last_motion_ms
            << ",\"first_damping_ms\":" << ev.first_damping_ms
            << ",\"damping_delay_ms\":"
            << (ev.damping_sent && ev.first_damping_ms >= ev.stop_ms ? (ev.first_damping_ms - ev.stop_ms) : 0)
            << ",\"physical_settled_ms\":" << physical_settled_ms
            << ",\"final_max_vel\":" << last_max_vel
            << ",\"tx_motion_frames\":" << tx_motion_count_.load()
            << ",\"tx_damping_frames\":" << tx_damping_count_.load();
        log_event("stop_test_summary", sum.str());
        return 0;
    }

    void release_sdk_fast() {
        {
            std::lock_guard<std::mutex> glock(guard_call_mu_);
            sdk_authority_.store(false);
            guard_armed_.store(false);
        }
        if (sender_) {
            // Send exact exit-SDK command 0x113 immediately without ControlGet(ROBOT)'s 2-second sleep.
            sender_->SetCmd(0x113, 0);
            log_event("sdk_authority_released", "\"mode\":\"ROBOT\",\"fast_0x113\":true");
        }
    }

    template<class RefreshFn>
    bool execute_native_stand_handoff(const Profile&, bool was_sdk_active, RefreshFn&& refresh_hold) {
        if (stop_latched_.load() || cancel_requested_.load()) {
            if (was_sdk_active) release_sdk_fast();
            log_event("native_handoff_aborted", "\"reason\":\"stop_before_handoff\"");
            return false;
        }

        // Wait briefly while SDK STILL HOLDS PRE_POSE (keeping Guard fresh every 5 ms -> zero drop!).
        const Millis wait_end = steady_ms() + 800;
        while (!factory_waiting_standup_.load() && steady_ms() < wait_end
               && !stop_latched_.load() && !cancel_requested_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (was_sdk_active) refresh_hold();
        }
        if (stop_latched_.load() || cancel_requested_.load()) {
            if (was_sdk_active) release_sdk_fast();
            log_event("native_handoff_aborted", "\"reason\":\"stop_before_stand_cmd\"");
            return false;
        }

        // Seamless zero-drop handoff: send stand_cmd (0x21010202) FIRST while SDK
        // is still holding PRE_POSE, wait until jy_exe enters "Standing up, please wait..."
        // (~5 ms, actively commanding PRE_POSE), and ONLY THEN send 0x113 to switch
        // motor control directly from SDK PRE_POSE to jy_exe Standing up!
        factory_standing_started_.store(false);
        factory_force_control_.store(false);
        sender_->SetCmd(0x21010202, 0);
        const Millis stand_cmd_ms = steady_ms();
        log_event("native_stand_cmd_sent", "\"code\":\"0x21010202\",\"pre_release\":true");

        if (was_sdk_active) {
            const Millis start_ack_deadline = steady_ms() + 250;
            while (!factory_standing_started_.load() && steady_ms() < start_ack_deadline
                   && !stop_latched_.load() && !cancel_requested_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                if (!factory_standing_started_.load()) refresh_hold();
            }
            log_event("native_handoff_releasing_sdk",
                      "\"mode\":\"ROBOT\",\"standing_pre_ack\":"
                      + std::string(factory_standing_started_.load() ? "true" : "false"));
            release_sdk_fast();
        }

        bool retried_stand_cmd = false;
        Millis upright_since = 0;
        const Millis timeout_ms = stand_cmd_ms + 7000;
        while (!shutdown_.load() && steady_ms() < timeout_ms) {
            const Millis now = steady_ms();
            const auto s = get_hw_snapshot();
            const auto quat = rpy_deg_to_quat_xyzw(s.rpy_deg[0], s.rpy_deg[1], s.rpy_deg[2]);
            const double cur_tilt = tilt(quat);
            const double cur_h = estimate_base_height_m(s.pos, quat);
            const Millis elapsed = now - stand_cmd_ms;

            if (stop_latched_.load() || cancel_requested_.load()) {
                sender_->SetCmd(0x21020C0E, 0); // Factory UDP STOP
                log_event("native_stand_stopped", "\"code\":\"0x21020C0E\",\"reason\":\"operator_stop\"");
                return false;
            }

            // Fallback: if jy_exe did not accept pre-release stand_cmd, resend once after release.
            if (!retried_stand_cmd && !factory_standing_started_.load() && elapsed >= 400) {
                retried_stand_cmd = true;
                sender_->SetCmd(0x21010202, 0);
                log_event("native_stand_cmd_resent", "\"code\":\"0x21010202\",\"elapsed_ms\":" + std::to_string(elapsed));
            }

            // Check upright + force control completion.
            if (cur_tilt <= 15.0 && cur_h >= 0.22) {
                if (upright_since == 0) upright_since = now;
                if (now - upright_since >= 800 && (factory_force_control_.load() || elapsed >= 2800)) {
                    std::ostringstream ex;
                    ex << std::fixed << std::setprecision(2)
                       << "\"tilt_deg\":" << cur_tilt
                       << ",\"est_height_m\":" << std::setprecision(3) << cur_h
                       << ",\"force_control_seen\":" << (factory_force_control_.load() ? "true" : "false")
                       << ",\"elapsed_ms\":" << elapsed;
                    log_event("native_stand_complete_handset_ready", ex.str());
                    return true;
                }
            } else {
                upright_since = 0;
                // Only declare stand-up obstructed AFTER factory stand-up should have completed.
                if (factory_force_control_.load() && elapsed >= 2800 && (cur_tilt > 20.0 || cur_h < 0.20)) {
                    sender_->SetCmd(0x21020C0E, 0);
                    log_event("native_stand_stopped", "\"code\":\"0x21020C0E\",\"reason\":\"not_upright_after_stand\"");
                    return false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        sender_->SetCmd(0x21020C0E, 0);
        log_event("native_stand_stopped", "\"code\":\"0x21020C0E\",\"reason\":\"timeout\"");
        return false;
    }

    int run_recover(uint64_t session_id, const Profile& profile, Millis start_ms) {
        if (!opt_.continuous) {
            fs::create_directories(opt_.lock_dir);
            ::chmod(opt_.lock_dir.c_str(), 0700);
            if (!reserve_trial(opt_.lock_dir, session_id)) {
                throw std::runtime_error(
                    "Durable lock exists in " + opt_.lock_dir + "/unfinished-sdk-recovery; review & archive first");
            }
            log_event("trial_reserved", "\"lock_dir\":\"" + opt_.lock_dir + "\"");
        }

        sender_ = std::make_unique<Sender>(opt_.robot_ip, static_cast<uint16_t>(opt_.robot_port));
        // NOTE: Never call sender_->RobotStateInit()!
        start_guard_tx_thread();

        const Millis global_deadline = opt_.continuous
            ? std::numeric_limits<Millis>::max()
            : (start_ms + static_cast<Millis>(opt_.duration_s) * 1000ULL);
        uint64_t cycle_idx = 0;
        int last_rc = 0;

        const Profile right_profile = mirror_profile_right(profile);
        if (!valid_trajectory(right_profile.start, right_profile.segments, right_profile.bounds)) {
            throw std::runtime_error("Mirrored right-roll profile failed trajectory validation");
        }

        while (!shutdown_.load() && !cancel_requested_.load() && steady_ms() < global_deadline) {
            ++cycle_idx;
            const uint64_t attempt_session = session_id + cycle_idx;

            SequenceConfig seq_cfg{};
            seq_cfg.fall_tilt_deg = 45.0; // Catches side-lying falls (>=45 deg) while >35 deg above normal walking
            seq_cfg.supine_tilt_deg = 45.0; // Both side-lying (45..120 deg) and supine (>=120 deg) use directional roll!
            seq_cfg.detect_dwell_ms = 300;
            seq_cfg.stable_dwell_ms = opt_.stable_wait_ms;
            seq_cfg.allow_prone_shortcut = opt_.allow_prone_shortcut;

            // Default controller uses left profile; if robot settles right-side-down
            // (-165 < roll < 0) before rolling starts, we hot-swap to right_profile.
            bool using_right_profile = false;
            const Profile* active_profile = &profile;
            {
                std::lock_guard<std::mutex> glock(guard_call_mu_);
                sdk_authority_.store(false);
                guard_armed_.store(false);
                recovery_active_.store(false);
                stop_latched_.store(false);
                stop_reason_.store(static_cast<int>(StopReason::none));
                stop_ms_.store(0);
                logged_first_motion_.store(false);
                logged_first_damping_.store(false);
                tx_motion_count_.store(0);
                tx_damping_count_.store(0);
                factory_waiting_standup_.store(false);
                factory_standing_started_.store(false);
                factory_force_control_.store(false);

                controller_ = std::make_unique<Controller>(profile, seq_cfg);
                Commissioning checks{true, true, true, true};
                if (!controller_->enable(attempt_session, checks, true)) {
                    throw std::runtime_error("Controller::enable() rejected profile/commissioning");
                }
                active_guard_ = &controller_->guard();
            }

            OfficialStandUpConfig stand_cfg{};
            stand_cfg.kp_scale = opt_.kp_scale;
            OfficialStandUp standup(stand_cfg);

            Phase last_phase = Phase::watching;
            bool logged_fall_detected = false;
            bool cycle_succeeded = false;
            bool cycle_aborted = false;
            uint64_t last_watch_estop_epoch = watching_estop_epoch_.load();
            Millis stopped_at_ms = 0;
            Millis upright_at_ms = 0;
            Millis settled_since = 0;
            Millis physical_settled_ms = 0;
            uint64_t loop_cnt = 0;
            uint64_t next_ns = steady_ns();

            log_event("observing",
                      "\"cycle\":" + std::to_string(cycle_idx)
                      + ",\"stable_wait_ms\":" + std::to_string(opt_.stable_wait_ms)
                      + ",\"detect_dwell_ms\":300"
                      + ",\"stand_mode\":\"" + opt_.stand_mode + "\"");

            while (!shutdown_.load() && !cancel_requested_.load() && steady_ms() < global_deadline) {
                next_ns += 5000000ULL; // 200 Hz
                const auto s = get_hw_snapshot();
                const auto quat = rpy_deg_to_quat_xyzw(s.rpy_deg[0], s.rpy_deg[1], s.rpy_deg[2]);
                Feedback fb{s.seq, s.mono_ms, s.pos, s.vel, quat};
                const auto prone = assess_prone_support(fb, s.contact_z, standup.pre_pose());

                // If an operator handset STOP happened while passively watching, reset fall timer for 1 tick.
                const uint64_t cur_estop_epoch = watching_estop_epoch_.load();
                const bool estop_reset_tick = (last_phase == Phase::watching && cur_estop_epoch != last_watch_estop_epoch);
                last_watch_estop_epoch = cur_estop_epoch;

                // While passively watching (not yet rolling), pick left vs right mirrored
                // roll profile based on which side the robot is lying on:
                //   -165 < roll < 0  => right side down => right_profile (pushes FR/HR)
                //   otherwise        => left side down or pure supine => left_profile (pushes FL/HL)
                if (last_phase == Phase::watching && tilt(quat) >= seq_cfg.fall_tilt_deg) {
                    const double roll_deg = s.rpy_deg[0];
                    const bool want_right = (roll_deg < 0.0 && roll_deg > -165.0);
                    if (want_right != using_right_profile) {
                        std::lock_guard<std::mutex> glock(guard_call_mu_);
                        using_right_profile = want_right;
                        active_profile = using_right_profile ? &right_profile : &profile;
                        controller_ = std::make_unique<Controller>(*active_profile, seq_cfg);
                        Commissioning checks{true, true, true, true};
                        controller_->enable(attempt_session, checks, true);
                        active_guard_ = &controller_->guard();
                        logged_fall_detected = false;
                    }
                }

                Millis now = 0;
                Status st{};
                {
                    std::lock_guard<std::mutex> glock(guard_call_mu_);
                    now = steady_ms();
                    Input in{};
                    in.feedback = fb;
                    in.exclusive_authority = true;
                    in.fall_allowed = !stop_latched_.load() && !estop_reset_tick;
                    in.prone_confirmed = prone.confirmed;
                    in.expects_progress = standup.expects_progress(now);
                    in.stand_complete = standup.complete(now);
                    if (stop_latched_.load()) {
                        in.stop = static_cast<StopReason>(stop_reason_.load());
                    }

                    const double stand_entry_kp = active_profile->segments.back().target[0].kp;
                    st = controller_->advance(now, in, [&](Phase phase, Joints& out) {
                        return standup.compute(now, fb, stand_entry_kp, phase, out);
                    });
                }

                if (st.fall_detected && !logged_fall_detected) {
                    logged_fall_detected = true;
                    std::ostringstream ex;
                    ex << "\"fall_detected_ms\":" << st.fall_detected_ms
                       << ",\"tilt_deg\":" << std::fixed << std::setprecision(2) << tilt(quat)
                       << ",\"roll_deg\":" << s.rpy_deg[0]
                       << ",\"direction\":\"" << (using_right_profile ? "right" : "left") << "\"";
                    log_event("fall_detected", ex.str());
                } else if (!st.fall_detected && last_phase == Phase::watching) {
                    logged_fall_detected = false;
                }

                if (st.phase != last_phase) {
                    std::ostringstream ex;
                    ex << "\"from\":\"" << phase_name(last_phase) << "\""
                       << ",\"to\":\"" << phase_name(st.phase) << "\""
                       << ",\"stop\":\"" << stop_reason_name(st.stop) << "\""
                       << ",\"direction\":\"" << (using_right_profile ? "right" : "left") << "\""
                       << ",\"tilt_deg\":" << std::fixed << std::setprecision(2) << tilt(quat);
                    log_event("phase_changed", ex.str());

                    if (last_phase == Phase::watching
                        && (st.phase == Phase::rolling || st.phase == Phase::prone_check)) {
                        recovery_active_.store(true);
                        if (st.phase == Phase::rolling || opt_.stand_mode == "sdk") {
                            guard_armed_.store(true);
                            sender_->ControlGet(SDK);
                            sdk_authority_.store(true);
                            log_event("sdk_authority_acquired", "\"mode\":\"SDK\"");
                        }
                    }
                    last_phase = st.phase;
                }

                if (loop_cnt % 10 == 0) { // 20 Hz telemetry audit + ROS pub
                    publish_ros_telemetry(s);
                    std::ostringstream ex;
                    ex << std::fixed << std::setprecision(2)
                       << "\"phase\":\"" << phase_name(st.phase) << "\""
                       << ",\"tilt_deg\":" << tilt(quat)
                       << ",\"est_height_m\":" << std::setprecision(3) << prone.est_height_m
                       << ",\"prone_confirmed\":" << (prone.confirmed ? "true" : "false")
                       << ",\"pos\":" << json_axis(s.pos);
                    log_event("sample", ex.str(), false);
                }

                // Native stand-up handoff: as soon as prone_check confirms belly-down
                // and Sequence enters standing, hand back to jy_exe + send stand_cmd
                // so robot stands in full force-control mode with immediate handset control!
                if (st.phase == Phase::standing && opt_.stand_mode == "native-handoff") {
                    auto refresh_hold = [&]() {
                        const auto hs = get_hw_snapshot();
                        const auto hq = rpy_deg_to_quat_xyzw(hs.rpy_deg[0], hs.rpy_deg[1], hs.rpy_deg[2]);
                        Feedback hfb{hs.seq, hs.mono_ms, hs.pos, hs.vel, hq};
                        std::lock_guard<std::mutex> glock(guard_call_mu_);
                        const Millis hnow = steady_ms();
                        Input hin{};
                        hin.feedback = hfb;
                        hin.exclusive_authority = true;
                        hin.prone_confirmed = true;
                        if (stop_latched_.load()) {
                            hin.stop = static_cast<StopReason>(stop_reason_.load());
                        }
                        controller_->advance(hnow, hin, [&](Phase, Joints& out) {
                            // Keep holding PRE_POSE smoothly with zero velocity while awaiting jy_exe pre-ack.
                            out = active_profile->segments.back().target;
                            return true;
                        });
                    };
                    const bool ok = execute_native_stand_handoff(*active_profile, sdk_authority_.load(), refresh_hold);
                    if (ok) {
                        cycle_succeeded = true;
                        last_phase = Phase::upright_hold;
                    } else {
                        cycle_aborted = true;
                        last_phase = Phase::stopped;
                    }
                    break;
                }

                if (st.phase == Phase::stopped) {
                    cycle_aborted = true;
                    if (stopped_at_ms == 0) stopped_at_ms = now;
                    double max_v = 0.0;
                    for (double v : s.vel) max_v = std::max(max_v, std::abs(v));
                    if (max_v < 0.15) {
                        if (settled_since == 0) settled_since = now;
                        if (physical_settled_ms == 0 && now - settled_since >= 300) {
                            physical_settled_ms = now;
                            log_event("physical_settled", "\"settled_ms\":" + std::to_string(now));
                        }
                    } else {
                        settled_since = 0;
                    }
                    if (now - stopped_at_ms >= static_cast<Millis>(opt_.damping_hold_s * 1000.0)) {
                        break;
                    }
                } else if (st.phase == Phase::upright_hold) {
                    cycle_succeeded = true;
                    if (upright_at_ms == 0) upright_at_ms = now;
                    if (now - upright_at_ms >= static_cast<Millis>(opt_.stand_hold_s * 1000.0)) {
                        break;
                    }
                }

                ++loop_cnt;
                sleep_until_ns(next_ns);
            }

            // Ensure SDK authority is cleanly and IMMEDIATELY released after every cycle.
            if (sdk_authority_.load()) {
                release_sdk_fast();
            }
            recovery_active_.store(false);

            const auto ev = controller_->guard().evidence();
            std::ostringstream sum;
            sum << "\"cycle\":" << cycle_idx
                << ",\"final_phase\":\"" << phase_name(last_phase) << "\""
                << ",\"stop_reason\":\"" << stop_reason_name(ev.reason) << "\""
                << ",\"stop_ms\":" << ev.stop_ms
                << ",\"last_motion_ms\":" << ev.last_motion_ms
                << ",\"first_damping_ms\":" << ev.first_damping_ms
                << ",\"damping_delay_ms\":"
                << (ev.damping_sent && ev.first_damping_ms >= ev.stop_ms ? (ev.first_damping_ms - ev.stop_ms) : 0)
                << ",\"physical_settled_ms\":" << physical_settled_ms
                << ",\"tx_motion_frames\":" << tx_motion_count_.load()
                << ",\"tx_damping_frames\":" << tx_damping_count_.load();
            log_event("recover_summary", sum.str());
            last_rc = cycle_succeeded ? 0 : 1;

            if (!opt_.continuous || cancel_requested_.load() || shutdown_.load()) {
                break;
            }

            // Continuous mode safety:
            //   - After an abort/STOP/stall, DO NOT retry the same fall! Wait passively
            //     until an operator restores the robot upright (tilt <= 15 deg, h >= 0.22m
            //     for 3.0s) before re-arming for future falls.
            //   - After a successful recovery (already verified upright 0.8s), wait 0.5s
            //     upright before re-arming for the next fall.
            if (cycle_aborted) {
                log_event("paused_after_stop_waiting_upright",
                          "\"detail\":\"manual_upright_3s_required_to_rearm\"");
            }
            const Millis required_upright_ms = cycle_aborted ? 3000 : 500;
            Millis upright_hold_since = 0;
            while (!shutdown_.load() && !cancel_requested_.load()) {
                const Millis now = steady_ms();
                const auto s = get_hw_snapshot();
                const auto q = rpy_deg_to_quat_xyzw(s.rpy_deg[0], s.rpy_deg[1], s.rpy_deg[2]);
                const double t_deg = tilt(q);
                const double h_m = estimate_base_height_m(s.pos, q);
                if (t_deg <= 15.0 && h_m >= 0.22) {
                    if (upright_hold_since == 0) upright_hold_since = now;
                    if (now - upright_hold_since >= required_upright_ms) {
                        log_event(cycle_aborted ? "rearmed_after_manual_upright" : "rearmed_after_recovery",
                                  "\"tilt_deg\":" + std::to_string(t_deg));
                        break;
                    }
                } else {
                    upright_hold_since = 0;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        return last_rc;
    }

    void log_event(const char* event, const std::string& extra_fields, bool echo = true) {
        std::ostringstream os;
        os << "{\"event\":\"" << event << "\",\"mono_ms\":" << steady_ms();
        if (!extra_fields.empty()) os << ',' << extra_fields;
        os << '}';
        audit_.write_line(os.str(), echo);
    }

    Options opt_;
    AuditLog audit_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<drdds::msg::JointsData>::SharedPtr joints_pub_;
    rclcpp::Publisher<drdds::msg::ImuData>::SharedPtr imu_pub_;
    rclcpp::Publisher<drdds::msg::GamepadData>::SharedPtr gamepad_pub_;
    rclcpp::Service<drdds::srv::StdSrvInt32>::SharedPtr estop_srv_;

    std::unique_ptr<Receiver> receiver_;
    std::unique_ptr<Sender> sender_;
    std::shared_ptr<RetroidGamepad> gamepad_;
    std::unique_ptr<DeepRcsLogWatcher> deeprcs_watcher_;
    std::unique_ptr<Controller> controller_;
    std::unique_ptr<Guard> standalone_guard_;
    Guard* active_guard_{nullptr};

    mutable std::mutex hw_mu_;
    std::mutex guard_call_mu_;
    uint64_t hw_seq_{0};
    HwSnapshot latest_hw_{};

    std::atomic<bool> shutdown_{false};
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> recovery_active_{false};
    std::atomic<bool> guard_armed_{false};
    std::atomic<bool> sdk_authority_{false};
    std::atomic<bool> stop_latched_{false};
    std::atomic<bool> logged_first_motion_{false};
    std::atomic<bool> logged_first_damping_{false};
    std::atomic<bool> factory_waiting_standup_{false};
    std::atomic<bool> factory_standing_started_{false};
    std::atomic<bool> factory_force_control_{false};
    std::atomic<uint64_t> watching_estop_epoch_{0};
    std::atomic<int> stop_reason_{static_cast<int>(StopReason::none)};
    std::atomic<Millis> stop_ms_{0};
    std::atomic<uint16_t> last_gamepad_buttons_{0};
    std::atomic<uint64_t> tx_motion_count_{0};
    std::atomic<uint64_t> tx_damping_count_{0};

    std::thread tx_thread_;
    std::thread ros_thread_;
    std::thread watcher_thread_;
    std::thread stdin_thread_;
};

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        Options opt{};
        if (!parse_args(argc, argv, opt)) return 0;
        rclcpp::init(argc, argv);
        InterruptibleRecoveryApp app(opt);
        const int rc = app.run();
        rclcpp::shutdown();
        return rc;
    } catch (const std::exception& exc) {
        std::cerr << "lite3_interruptible_recovery error: " << exc.what() << std::endl;
        if (rclcpp::ok()) rclcpp::shutdown();
        return 2;
    }
}
