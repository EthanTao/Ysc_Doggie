#include "lite3_recovery/controller.hpp"
#include "lite3_recovery/reservation.hpp"

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <thread>

using namespace lite3_recovery;

#define CHECK(expr) do { if (!(expr)) { std::cerr << __LINE__ << ": " #expr "\n"; std::exit(1); } } while(false)

static Axis uniform(double value) {
    Axis a;
    a.fill(value);
    return a;
}

static Bounds test_bounds() {
    return {uniform(-1.),uniform(1.),uniform(1.),uniform(10.),uniform(2.),
            uniform(20.),uniform(10.),uniform(20.)};
}

constexpr Commissioning mock_commissioned{true,true,true,true};
constexpr StallLimits mock_stall{300,.02,.08,2.};

struct Fixture {
    Guard guard;
    Frame cmd{42,1,100,{}};
    Fixture() {
        CHECK(guard.arm(42,100,mock_commissioned,true,test_bounds(),uniform(.5)));
        CHECK(guard.feedback(1,100,100,{},{}));
    }
};

static bool verify_damping(Output type, const Joints& command) {
    CHECK(type==Output::damping);
    for(const auto& j : command) CHECK(j.position==0 && j.velocity==0 && j.kp==0 && j.kd==.5 && j.feedforward==0);
    return true;
}

static bool accept_motion(Output type,const Joints&) {
    CHECK(type==Output::motion);
    return true;
}

static void guard_tests() {
    Guard disabled;
    CHECK(!disabled.arm(42,100,{},true,test_bounds(),uniform(.5)));
    CHECK(!disabled.arm(42,100,mock_commissioned,false,test_bounds(),uniform(.5)));
    CHECK(disabled.tick(100,true,accept_motion)==Output::none);

    for (const auto reason : {StopReason::handset,StopReason::cancel,StopReason::stall,
                              StopReason::fault,StopReason::timeout}) {
        Fixture f;
        CHECK(f.guard.accept(f.cmd,100));
        CHECK(f.guard.tick(101,true,accept_motion)==Output::motion);
        f.guard.stop(reason,102);
        ++f.cmd.sequence;
        f.cmd.source_ms=102;
        CHECK(!f.guard.accept(f.cmd,102));
        CHECK(f.guard.tick(103,true,verify_damping)==Output::damping);
        CHECK(f.guard.tick(104,true,verify_damping)==Output::damping);
        auto e=f.guard.evidence();
        CHECK(e.reason==reason && e.stop_ms==102 && e.last_motion_ms==101 && e.first_damping_ms==103);
        CHECK(!f.guard.arm(99,105,mock_commissioned,true,test_bounds(),uniform(.5)));
    }
    {
        Fixture f;
        CHECK(f.guard.accept(f.cmd,100));
        CHECK(!f.guard.accept(f.cmd,109));
        f.cmd.session=41;
        ++f.cmd.sequence;
        f.cmd.source_ms=110;
        CHECK(!f.guard.accept(f.cmd,110));
        CHECK(f.guard.tick(119,true,accept_motion)==Output::motion);
        CHECK(f.guard.tick(120,true,verify_damping)==Output::damping);
        CHECK(f.guard.evidence().reason==StopReason::command_expired);
        f.cmd.session=42;
        f.cmd.source_ms=121;
        CHECK(!f.guard.accept(f.cmd,121));
    }
    {
        Fixture f;
        for (Millis i=100;i<300;i+=5) {
            f.cmd.source_ms=i;
            CHECK(f.guard.accept(f.cmd,i));
            CHECK(f.guard.tick(i,true,accept_motion)==Output::motion);
            ++f.cmd.sequence;
        }
        CHECK(!f.guard.feedback(1,299,299,{},{})); // Republished frame must not refresh watchdog
        CHECK(f.guard.tick(300,true,verify_damping)==Output::damping);
        CHECK(f.guard.evidence().reason==StopReason::feedback);
    }
    {
        Fixture f;
        CHECK(f.guard.accept(f.cmd,100));
        CHECK(f.guard.tick(105,false,accept_motion)==Output::none);
        CHECK(f.guard.tick(106,true,accept_motion)==Output::none);
        CHECK(f.guard.evidence().reason==StopReason::ownership);
    }
    {
        Fixture f;
        f.cmd.joints[0].position=std::numeric_limits<double>::quiet_NaN();
        CHECK(!f.guard.accept(f.cmd,100));
        CHECK(f.guard.tick(101,true,verify_damping)==Output::damping);
        CHECK(f.guard.evidence().reason==StopReason::fault);
    }
    {
        Fixture f;
        f.cmd.source_ms=104;
        CHECK(!f.guard.accept(f.cmd,103));
        CHECK(f.guard.tick(102,true,verify_damping)==Output::damping);
        CHECK(f.guard.evidence().reason==StopReason::clock);
    }
    {
        Fixture f;
        CHECK(f.guard.accept(f.cmd,100));
        auto failed=[](Output type,const Joints& j) {
            if (type==Output::motion) return false;
            return verify_damping(type,j);
        };
        CHECK(f.guard.tick(101,true,failed)==Output::damping);
        CHECK(f.guard.evidence().reason==StopReason::transport);
    }
    {
        Fixture f;
        CHECK(f.guard.accept(f.cmd,100));
        f.cmd.source_ms=105;
        ++f.cmd.sequence;
        f.cmd.joints[0].position=.2; // individually finite/in range, but unsafe step
        CHECK(!f.guard.accept(f.cmd,105));
        CHECK(f.guard.tick(105,true,verify_damping)==Output::damping);
    }
    {
        Fixture f;
        CHECK(f.guard.feedback(2,105,105,uniform(3.),{}));
        f.cmd.source_ms=105;
        f.cmd.joints[0].kp=10;
        CHECK(f.guard.accept(f.cmd,105));
        CHECK(f.guard.tick(105,true,verify_damping)==Output::damping);
        CHECK(f.guard.evidence().reason==StopReason::fault);
    }
    {
        // Try publishing on a second thread after the actual shared boundary
        // has latched. This covers the cache race, not the robot's response.
        Fixture f;
        CHECK(f.guard.accept(f.cmd,100));
        std::atomic<bool> latched{false};
        std::thread stop([&] {f.guard.stop(StopReason::handset,105); latched.store(true);});
        std::thread send([&] {
            while (!latched.load()) std::this_thread::yield();
            CHECK(f.guard.tick(106,true,verify_damping)==Output::damping);
        });
        stop.join();
        send.join();
        CHECK(!f.guard.evidence().motion_sent);
    }
}

static Status step(Sequence& seq, Input& input, Millis now) {
    ++input.feedback.sequence;
    input.feedback.received_ms=now;
    return seq.tick(now,input);
}

static Status roll(Sequence& seq, Input& input) {
    CHECK(seq.enable(mock_commissioned,true));
    input.exclusive_authority=input.fall_allowed=true;
    input.feedback.attitude={1,0,0,0};
    for (Millis i=100;i<5100;i+=10) CHECK(step(seq,input,i).phase==Phase::watching);
    return step(seq,input,5100);
}

static void sequence_tests() {
    {
        Sequence seq(mock_stall);
        Input in;
        CHECK(roll(seq,in).phase==Phase::rolling);
        in.stop=StopReason::handset;
        in.roll_complete=true;
        CHECK(step(seq,in,5110).stop==StopReason::handset);
        in.stop=StopReason::none;
        CHECK(step(seq,in,5120).phase==Phase::stopped);
    }
    {
        Sequence seq(mock_stall);
        Input in;
        CHECK(roll(seq,in).phase==Phase::rolling);
        in.expects_progress=true;
        for (Millis i=5110;i<5410;i+=10) CHECK(step(seq,in,i).phase==Phase::rolling);
        CHECK(step(seq,in,5410).stop==StopReason::stall);
    }
    {
        Sequence seq(mock_stall);
        Input in;
        CHECK(roll(seq,in).phase==Phase::rolling);
        // Planned pause is excluded, even if it outlasts the stall window.
        for (Millis i=5110;i<6000;i+=10) CHECK(step(seq,in,i).phase==Phase::rolling);
        in.roll_complete=true;
        in.feedback.attitude={0,0,0,1};
        CHECK(step(seq,in,6000).phase==Phase::prone_check);
        for (Millis i=6010;i<6600;i+=10) CHECK(step(seq,in,i).phase==Phase::prone_check);
        in.prone_confirmed=true;
        for (Millis i=6600;i<7100;i+=10) CHECK(step(seq,in,i).phase==Phase::prone_check);
        CHECK(step(seq,in,7100).phase==Phase::standing);
        in.stand_complete=true;
        in.stop=StopReason::cancel;
        CHECK(step(seq,in,7110).stop==StopReason::cancel);
    }
    {
        Sequence seq(mock_stall);
        Input in;
        CHECK(roll(seq,in).phase==Phase::rolling);
        CHECK(seq.tick(5300,in).stop==StopReason::feedback);
    }
    {
        Sequence seq(mock_stall);
        Input in;
        CHECK(roll(seq,in).phase==Phase::rolling);
        for (Millis i=5110;i<15100;i+=10) CHECK(step(seq,in,i).phase==Phase::rolling);
        CHECK(step(seq,in,15100).stop==StopReason::timeout);
    }
}

static void trajectory_tests() {
    Joints start{};
    Segment s{1000,{},true};
    s.target[0].position=.3;
    s.target[0].kp=2;
    const auto limits=test_bounds();
    CHECK(valid_trajectory(start,{s},limits));
    CHECK(interpolate(start,s,0)[0].position==0.);
    CHECK(interpolate(start,s,1000)[0].position==.3);
    CHECK(interpolate(start,s,1000)[0].velocity==0.);
    CHECK(std::abs(interpolate(start,s,500)[0].velocity-.5625)<1e-9);
    s.duration_ms=50;
    CHECK(!valid_trajectory(start,{s},limits));
    s.duration_ms=1000;
    s.target[0].feedforward=.01;
    CHECK(!valid_trajectory(start,{s},limits));
    s.target[0].feedforward=0.;
    s.expects_progress=false;
    CHECK(!valid_trajectory(start,{s},limits));
}


static void controller_boundary_integration() {
    Segment roll_segment{1000,{},true};
    roll_segment.target[0].position=.2;
    Controller controller(Profile{test_bounds(),{}, {roll_segment},mock_stall,uniform(.5),.05});
    CHECK(controller.enable(12,mock_commissioned,true));
    Input in;
    in.exclusive_authority=in.fall_allowed=true;
    in.feedback.attitude={1,0,0,0};
    const auto no_stand=[](Phase,Joints&) {return false;};
    for (Millis i=100;i<=5150;i+=5) {
        ++in.feedback.sequence;
        in.feedback.received_ms=i;
        in.feedback.position[0]=(i>5100 ? .01 : 0.);
        if (i>5100) in.feedback.velocity[0]=.2;
        const auto state=controller.advance(i,in,no_stand);
        CHECK(state.phase==(i<5100 ? Phase::watching : Phase::rolling));
        CHECK(controller.guard().tick(i,true,accept_motion)==(i<5100 ? Output::none : Output::motion));
    }
    // Controller ceases ticking, but bridge keeps running independently.
    CHECK(controller.guard().tick(5169,true,accept_motion)==Output::motion);
    CHECK(controller.guard().tick(5170,true,verify_damping)==Output::damping);
    CHECK(controller.guard().evidence().last_motion_ms==5169);
    CHECK(controller.guard().evidence().first_damping_ms==5170);
    CHECK(controller.advance(5171,in,no_stand).stop==StopReason::command_expired);
    CHECK(controller.guard().tick(5171,true,verify_damping)==Output::damping);
}


static void controller_preempts_official_stand_callback() {
    Segment move{1000,{},true};
    move.target[0].position=.2;
    Controller controller(Profile{test_bounds(),{}, {move},mock_stall,uniform(.5),.05});
    CHECK(controller.enable(12,mock_commissioned,true));
    Input in;
    in.exclusive_authority=in.fall_allowed=true;
    in.feedback.attitude={1,0,0,0};
    std::size_t standing_calls{};
    auto stand=[&](Phase phase,Joints& out) {
        CHECK(phase==Phase::standing);
        ++standing_calls;
        out=move.target;
        return true;
    };
    for (Millis i=100;i<=6610;i+=5) {
        ++in.feedback.sequence;
        in.feedback.received_ms=i;
        if (i>=5100) in.feedback.velocity[0]=.2;
        if (i>=6100) {
            in.feedback.attitude={0,0,0,1};
            in.prone_confirmed=true;
        }
        auto status=controller.advance(i,in,stand);
        CHECK(status.stop==StopReason::none);
        CHECK(controller.guard().tick(i,true,accept_motion)==(i<5100 ? Output::none : Output::motion));
    }
    CHECK(standing_calls>0);
    const auto before=standing_calls;
    in.stop=StopReason::handset;
    in.stand_complete=true;
    CHECK(controller.advance(6615,in,stand).stop==StopReason::handset);
    CHECK(controller.guard().tick(6615,true,verify_damping)==Output::damping);
    CHECK(standing_calls==before);
}

static void durable_restart_test() {
    char dir[]="/tmp/lite3-sdk-latch-test-XXXXXX";
    CHECK(::mkdtemp(dir)!=nullptr);
    CHECK(reserve_trial(dir,111));
    CHECK(!reserve_trial(dir,222));
    std::filesystem::remove_all(dir);
}

int main() {
    guard_tests();
    sequence_tests();
    trajectory_tests();
    controller_boundary_integration();
    controller_preempts_official_stand_callback();
    durable_restart_test();
    std::cout << "Guard, sequence, trajectory and transport-boundary tests passed\n";
}
