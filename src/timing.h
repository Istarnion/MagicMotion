#ifndef TIMING_H_
#define TIMING_H_

// TODO(istarnion): Support ARM with an alternative to __rdtscp

#include <stdint.h>
#include <time.h>
#include <x86intrin.h>

/*
 * Get the current thread-specific time stamp in nanoseconds
 */
static inline uint64_t
GetTimestamp(void)
{
    struct timespec time;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time);
    uint64_t result = time.tv_sec * 1000000000UL + time.tv_nsec;

    return result;
}

/*
 * Get the current CPU cycle counter value
 */
static inline uint64_t
GetClockCycleCount(void)
{
    uint64_t result = __rdtsc();
    return result;
}

typedef struct
{
    uint64_t time;
    uint64_t clock;
} Timinginfo;

static Timinginfo
StartTiming(void)
{
    Timinginfo result = (Timinginfo){
        .time = GetTimestamp(),
        .clock = GetClockCycleCount()
    };

    return result;
}

static void
EndTimingAndPrint(const Timinginfo *info, const char *title)
{
    uint64_t duration = GetTimestamp() - info->time;
    uint64_t cycles = GetClockCycleCount() - info->clock;

    const uint64_t ss = duration / 1000000000UL;
    duration -= ss * 1000000000UL;
    const uint64_t ms = duration / 1000000UL;
    duration -= ms * 1000000UL;
    const uint64_t ns = duration;

    printf("%s: %lu cycles (%lu s, %lu ms, and %lu ns)\n", title, cycles, ss, ms, ns);
}

typedef struct
{
    uint64_t last_time;
    double avg_time;
    uint64_t last_clock;
    double avg_clock;
    int count;
} LoopedAverageTimerinfo;

static LoopedAverageTimerinfo
StartLoopedaverageTiming(void)
{
    LoopedAverageTimerinfo result = { 0 };
    return result;
}

static void
StartLoopedAverageCapture(LoopedAverageTimerinfo *info)
{
    info->last_time = GetTimestamp();
    info->last_clock = GetClockCycleCount();
}

static void
EndLoopedAverageCapture(LoopedAverageTimerinfo *info)
{
    uint64_t now_time = GetTimestamp();
    uint64_t now_clock = GetClockCycleCount();

    uint64_t delta_time = now_time - info->last_time;
    uint64_t delta_clock = now_clock - info->last_clock;

    info->avg_time = (info->avg_time * info->count + delta_time) / (info->count+1);
    info->avg_clock = (info->avg_clock * info->count + delta_clock) / (info->count+1);

    info->last_time = now_time;
    info->last_clock = now_clock;
    ++info->count;
}

static void
EndLoopedAverageTimingAndPrint(const LoopedAverageTimerinfo *info, const char *title)
{
    uint64_t duration = (uint64_t)info->avg_time;
    uint64_t cycles = (uint64_t)info->avg_clock;

    const uint64_t ss = duration / 1000000000UL;
    duration -= ss * 1000000000UL;
    const uint64_t ms = duration / 1000000UL;
    duration -= ms * 1000000UL;
    const uint64_t ns = duration;

    printf("%s: %lu cycles (%lu s, %lu ms, and %lu ns)\n", title, cycles, ss, ms, ns);
}

#endif /* end of include guard: TIMING_H_ */

