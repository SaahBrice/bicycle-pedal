#ifndef PEDAL_TIMING_H
#define PEDAL_TIMING_H

#include <stdint.h>

typedef struct {
    uint32_t start_cycles; // [cycles] Counter at computation entry.
    unsigned int irq_key;  // [1] Previous interrupt state.
} PedalTimingStamp;

typedef struct {
    uint64_t total_cycles; // [cycles] Sum of measured intervals.
    uint32_t max_cycles;   // [cycles] Longest measured call.
    uint32_t calls;        // [1] Number of measured calls.
} PedalTimingStats;

typedef struct {
    uint64_t total_cycles; // [cycles] Total measured computation.
    double total_us;       // [us] Total at configured CPU clock.
    double average_cycles; // [cycles/call] Mean measured cost.
    double average_us;     // [us/call] Mean at configured CPU clock.
    uint32_t max_cycles;   // [cycles] Longest measured call.
    uint32_t calls;        // [1] Number of measured calls.
} PedalTimingSummary;

// Enable and validate the Cortex-M33 DWT cycle counter; returns 0 or -ENOTSUP.
int pedal_timing_init(void);

// Stop cycle counting between measurements; preserve an attached debugger's trace gate.
void pedal_timing_suspend(void);

// Start one interrupt-isolated algorithm measurement.
PedalTimingStamp pedal_timing_begin(void);

// End a measurement, add measured cycles to stats, and return those cycles.
uint32_t pedal_timing_end(PedalTimingStamp stamp, PedalTimingStats *stats);

// Add one previously measured call to another accumulator.
void pedal_timing_record(PedalTimingStats *stats, uint32_t cycles);

// Add all calls and cycles from one accumulator to another.
void pedal_timing_merge(PedalTimingStats *destination,
                        const PedalTimingStats *source);

// Convert cycle statistics to average cycles and microseconds per call.
int pedal_timing_summarize(const PedalTimingStats *stats,
                           PedalTimingSummary *summary);

#endif // PEDAL_TIMING_H
