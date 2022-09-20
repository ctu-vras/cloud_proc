#pragma once

#include <chrono>

namespace cloud_proc
{

class Timer
{
protected:
    typedef std::chrono::high_resolution_clock Clock;
    typedef Clock::time_point Time;
    typedef std::chrono::duration<double> Duration;
    Time start_;
public:
    Timer()
        : start_(Clock::now())
    {}
    Timer(const Time& s)
        : start_(s)
    {}
    Timer(const Timer& s)
        : start_(s.start_)
    {}
    void reset()
    {
      start_ = Clock::now();
    }
    double secondsElapsed() const
    {
        return std::chrono::duration_cast<Duration>(Clock::now() - start_).count();
    }
};

}
