#pragma once

#include <algorithm>

namespace tf {

// Fixed-step accumulator.
// * Frame time is clamped to maxFrameTime (pause, window stall, debugger);
//   the excess is counted as dropped time.
// * At most maxStepsPerFrame steps run per frame; any remainder beyond one
//   step is also dropped to avoid a "spiral of death".
// * alpha() is the step fraction for interpolating poses between the last two ticks.
class FixedStepClock {
public:
    explicit FixedStepClock(double step = 1.0 / 120.0) : m_step(step) {}

    void setStep(double step) { m_step = step; m_accumulator = std::min(m_accumulator, step); }
    double step() const { return m_step; }

    double maxFrameTime = 0.25;  // s
    int maxStepsPerFrame = 12;

    // Add frame time; returns the number of physics steps to run.
    int advance(double frameTime) {
        m_lastDropped = 0.0;
        if (frameTime > maxFrameTime) {
            m_lastDropped += frameTime - maxFrameTime;
            frameTime = maxFrameTime;
        }
        m_accumulator += std::max(0.0, frameTime);
        const int available = int(m_accumulator / m_step);
        int steps = available;
        if (steps > maxStepsPerFrame) {
            m_lastDropped += (steps - maxStepsPerFrame) * m_step;
            steps = maxStepsPerFrame;
        }
        m_accumulator = std::clamp(m_accumulator - available * m_step, 0.0, m_step);
        m_totalDropped += m_lastDropped;
        m_overloaded = m_lastDropped > 0.0;
        return steps;
    }

    void reset() { m_accumulator = 0.0; }

    double alpha() const { return m_accumulator / m_step; }
    double lastDropped() const { return m_lastDropped; }
    double totalDropped() const { return m_totalDropped; }
    bool overloaded() const { return m_overloaded; }

private:
    double m_step;
    double m_accumulator = 0.0;
    double m_lastDropped = 0.0;
    double m_totalDropped = 0.0;
    bool m_overloaded = false;
};

} // namespace tf
