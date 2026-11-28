#include "lite3_recovery/sequence.hpp"

#include <iostream>
#include <string>

using namespace lite3_recovery;

// Positions are measured feedback. No SDK sender or joint targets are created.
int main() {
    StallWindow window;
    const StallLimits reference{500,.02,.08,2.}; // only for offline comparison
    Feedback sample;
    std::string phase;
    std::string previous;
    std::size_t count{}, candidates{};
    Millis previous_ms{}, max_gap{};
    while (std::cin >> phase >> sample.received_ms) {
        for (auto& v : sample.attitude) std::cin >> v;
        for (auto& v : sample.position) std::cin >> v;
        for (auto& v : sample.velocity) std::cin >> v;
        sample.sequence=++count;
        if (!std::cin || !valid_feedback(sample,sample.received_ms)) return 2;
        if (phase!=previous) {
            window.reset();
        } else {
            if (sample.received_ms<=previous_ms) return 2;
            max_gap=std::max(max_gap,sample.received_ms-previous_ms);
        }
        if (window.stalled(sample,reference)) ++candidates;
        previous=phase;
        previous_ms=sample.received_ms;
    }
    if (!std::cin.eof() || count==0) return 2;
    std::cout << "samples=" << count << " max_gap_ms=" << max_gap
              << " shadow_stall_windows=" << candidates << "\n";
}
