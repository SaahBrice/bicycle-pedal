#include "timing/timing.h"

#include <errno.h>
#include <stddef.h>
#include <cmsis_core.h>
#include <zephyr/kernel.h>

#ifndef NRF_CONFIG_CPU_FREQ_MHZ
#error "The nRF CPU frequency is required to convert DWT cycles to time"
#endif

#define PEDAL_CPU_FREQUENCY_HZ ((uint32_t)NRF_CONFIG_CPU_FREQ_MHZ * 1000000u) // [Hz] Configured CPU clock.

/* ------------------------------------------------------------------ */
/* Creates: int pedal_timing_init(void)                                */
/*                                                                    */
/* Description:                                                       */
/*   Enables the DWT CPU cycle counter. A short probe rejects a       */
/*   counter that is not advancing.                                    */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: int - 0 on success or -ENOTSUP if DWT is unavailable       */
/* ------------------------------------------------------------------ */
int pedal_timing_init(void)
{
#if defined(DCB_DEMCR_TRCENA_Msk)
    DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
#else
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#endif
#if defined(DWT_CTRL_NOCYCCNT_Msk)
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u) {
        return -ENOTSUP;
    }
#endif
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t before = DWT->CYCCNT;
    for (volatile unsigned int i = 0u; i < 16u; ++i) {
    }
    if (DWT->CYCCNT == before) {
        return -ENOTSUP;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Creates: PedalTimingStamp pedal_timing_begin(void)                  */
/*                                                                    */
/* Description:                                                       */
/*   Locks interrupts, enables the counter if sleep stopped it, and   */
/*   reads the CPU cycle counter immediately                           */
/*   before the caller invokes one algorithm. The matching end call   */
/*   must always follow, including after an algorithm error.          */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: PedalTimingStamp - counter reading and interrupt key       */
/* ------------------------------------------------------------------ */
PedalTimingStamp pedal_timing_begin(void)
{
    const unsigned int key = irq_lock();
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return (PedalTimingStamp) {
        .start_cycles = DWT->CYCCNT,
        .irq_key = key,
    };
}

/* ------------------------------------------------------------------ */
/* Creates: uint32_t pedal_timing_end(PedalTimingStamp stamp,         */
/*                                PedalTimingStats *stats)             */
/*                                                                    */
/* Description:                                                       */
/*   Reads cycles before restoring interrupts. Unsigned subtraction   */
/*   handles a single 32-bit counter wrap. The measured interval      */
/*   includes the small cost of reading the counter.                  */
/*                                                                    */
/* Input:                                                             */
/*   stamp - value returned by the matching begin call                */
/*   stats - accumulator for one algorithm                             */
/*                                                                    */
/* Output: uint32_t - measured cycles for this call; updates stats    */
/* ------------------------------------------------------------------ */
uint32_t pedal_timing_end(PedalTimingStamp stamp, PedalTimingStats *stats)
{
    const uint32_t elapsed = DWT->CYCCNT - stamp.start_cycles;
    irq_unlock(stamp.irq_key);
    pedal_timing_record(stats, elapsed);
    return elapsed;
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_timing_record(PedalTimingStats *stats,         */
/*                                   uint32_t cycles)                 */
/*                                                                    */
/* Description:                                                       */
/*   Adds one measured algorithm call to an accumulator.             */
/*                                                                    */
/* Input:                                                             */
/*   stats - destination accumulator                                   */
/*   cycles - corrected cycle count of one call                       */
/*                                                                    */
/* Output: void - updates stats if supplied                            */
/* ------------------------------------------------------------------ */
void pedal_timing_record(PedalTimingStats *stats, uint32_t cycles)
{
    if (stats != NULL) {
        stats->total_cycles += cycles;
        stats->calls++;
        if (cycles > stats->max_cycles) {
            stats->max_cycles = cycles;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_timing_merge(PedalTimingStats *destination,    */
/*                                  const PedalTimingStats *source)    */
/*                                                                    */
/* Description:                                                       */
/*   Combines call counts and cycle totals from two replay segments. */
/*                                                                    */
/* Input:                                                             */
/*   destination - accumulator to update                              */
/*   source - completed segment to add                                */
/*                                                                    */
/* Output: void - updates destination if both arguments exist        */
/* ------------------------------------------------------------------ */
void pedal_timing_merge(PedalTimingStats *destination,
                        const PedalTimingStats *source)
{
    if (destination == NULL || source == NULL) {
        return;
    }
    destination->total_cycles += source->total_cycles;
    destination->calls += source->calls;
    if (source->max_cycles > destination->max_cycles) {
        destination->max_cycles = source->max_cycles;
    }
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_timing_summarize(const PedalTimingStats *stats,  */
/*                                     PedalTimingSummary *summary)    */
/*                                                                    */
/* Description:                                                       */
/*   Converts cycles into microseconds using the configured nRF CPU   */
/*   frequency. This is CPU computation time, excluding pacing,       */
/*   revolution aggregation, formatting, and UART transmission.      */
/*                                                                    */
/* Input:                                                             */
/*   stats - measured calls and cycles                                 */
/*   summary - destination for readable values                        */
/*                                                                    */
/* Output: int - 0 on success, -ENODATA if empty, or -EINVAL          */
/* ------------------------------------------------------------------ */
int pedal_timing_summarize(const PedalTimingStats *stats,
                           PedalTimingSummary *summary)
{
    if (stats == NULL || summary == NULL) {
        return -EINVAL;
    }
    if (stats->calls == 0u) {
        return -ENODATA;
    }
    *summary = (PedalTimingSummary) {
        .total_cycles = stats->total_cycles,
        .total_us = (double)stats->total_cycles * 1000000.0 /
                    (double)PEDAL_CPU_FREQUENCY_HZ,
        .average_cycles = (double)stats->total_cycles / stats->calls,
        .average_us = (double)stats->total_cycles * 1000000.0 /
                      ((double)PEDAL_CPU_FREQUENCY_HZ * stats->calls),
        .max_cycles = stats->max_cycles,
        .calls = stats->calls,
    };
    return 0;
}
