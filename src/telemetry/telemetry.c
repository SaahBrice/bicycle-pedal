#include "telemetry/telemetry.h"
#include "configuration/constants.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>

#define PEDAL_TELEMETRY_BUFFER_SIZE 192u // [bytes] UART message buffer capacity.

/* ------------------------------------------------------------------ */
/* Creates: static int send_formatted(PedalMessageLevel level,        */
/*                                    const char *format, ...)         */
/*                                                                    */
/* Description:                                                       */
/*   Formats one bounded message and hands it to communication.       */
/*   Rejects formatting errors and truncation.                         */
/*                                                                    */
/* Input:                                                             */
/*   level - output state                                              */
/*   format - printf-style format string                               */
/*   ... - matching format arguments                                   */
/*                                                                    */
/* Output:                                                            */
/*   int - 0 or negative errno code                                    */
/* ------------------------------------------------------------------ */
static int send_formatted(PedalMessageLevel level, const char *format, ...)
{
    char buffer[PEDAL_TELEMETRY_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0) {
        return -EIO;
    }
    if ((size_t)length >= sizeof(buffer)) {
        return -ENOSPC;
    }
    return pedal_communication_send(level, buffer);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_telemetry_status(PedalMessageLevel level,       */
/*                                     const char *message)            */
/* Description:                                                       */
/*   Forwards a readable message with its state.                       */
/*                                                                    */
/* Input:                                                             */
/*   level - plain, success, warning, or error                         */
/*   message - readable text                                           */
/*                                                                    */
/* Output:                                                            */
/*   int - communication status                                       */
/* ------------------------------------------------------------------ */
int pedal_telemetry_status(PedalMessageLevel level, const char *message)
{
    return pedal_communication_send(level, message);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_telemetry_error(const char *component,          */
/*                                    uint32_t sample_index, int code) */
/* Description:                                                       */
/*   Prints failure context without making the failing component      */
/*   depend on telemetry.                                               */
/*                                                                    */
/* Input:                                                             */
/*   component - failing stage name                                    */
/*   sample_index - current sample index                               */
/*   code - negative errno code                                        */
/*                                                                    */
/* Output:                                                            */
/*   int - communication or formatting status                         */
/* ------------------------------------------------------------------ */
int pedal_telemetry_error(const char *component, uint32_t sample_index,
                          int code)
{
    if (component == NULL) {
        return -EINVAL;
    }
    return send_formatted(PEDAL_MESSAGE_ERROR, "%s sample=%u code=%d",
                          component, (unsigned int)sample_index, code);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_telemetry_sample(const PedalSensorSample *sample, */
/*                                     const PedalResult *result)       */
/* Description:                                                       */
/*   Formats one simplified sample result with physical units.        */
/*                                                                    */
/* Input:                                                             */
/*   sample - source reading and timestamp                             */
/*   result - simplified algorithm output                              */
/*                                                                    */
/* Output:                                                            */
/*   int - communication or formatting status                         */
/* ------------------------------------------------------------------ */
int pedal_telemetry_sample(const PedalSensorSample *sample,
                           const PedalResult *result)
{
    if (sample == NULL || result == NULL) {
        return -EINVAL;
    }
    return send_formatted(PEDAL_MESSAGE_PLAIN,
                          "algorithm=simplified sample=%u t=%.4f [s] torque=%.5f [N*m] power=%.5f [W]",
                          (unsigned int)sample->sample_index,
                          (double)sample->time_s, (double)result->torque_Nm,
                          (double)result->power_W);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_telemetry_summary(const char *label,            */
/*                                      uint32_t turns,               */
/*                                      const PedalSummary *summary)   */
/* Description:                                                       */
/*   Formats one revolution or aggregate summary.                     */
/*                                                                    */
/* Input:                                                             */
/*   label - summary type and algorithm name                           */
/*   turns - completed revolution count                                */
/*   summary - sample means                                            */
/*                                                                    */
/* Output:                                                            */
/*   int - communication or formatting status                         */
/* ------------------------------------------------------------------ */
int pedal_telemetry_summary(const char *label, uint32_t turns,
                            const PedalSummary *summary)
{
    if (label == NULL || summary == NULL) {
        return -EINVAL;
    }
    return send_formatted(PEDAL_MESSAGE_PLAIN,
                          "%s turns=%u samples=%u mean_torque=%.6f [N*m] mean_power=%.6f [W]",
                          label, (unsigned int)turns, (unsigned int)summary->samples,
                          (double)summary->average_torque_Nm,
                          (double)summary->average_power_W);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_telemetry_comparison(float maximum_delta_Nm,    */
/*                                         float maximum_delta_W)      */
/* Description:                                                       */
/*   Formats the largest absolute deviations between standard and     */
/*   simplified results across the replay.                             */
/*                                                                    */
/* Input:                                                             */
/*   maximum_delta_Nm - maximum torque difference [N*m]               */
/*   maximum_delta_W - maximum power difference [W]                   */
/*                                                                    */
/* Output:                                                            */
/*   int - communication or formatting status                         */
/* ------------------------------------------------------------------ */
int pedal_telemetry_comparison(float maximum_delta_Nm, float maximum_delta_W)
{
    return send_formatted(PEDAL_MESSAGE_PLAIN,
                          "algorithm comparison max_torque_delta=%.9f [N*m] max_power_delta=%.9f [W]",
                          (double)maximum_delta_Nm, (double)maximum_delta_W);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_telemetry_timing(const char *scope,            */
/*                                    uint32_t revolution,            */
/*                                    const char *algorithm,           */
/*                                    const PedalTimingSummary        */
/*                                    *summary)                        */
/*                                                                    */
/* Description:                                                       */
/*   Prints total and per-sample corrected DWT cycles and CPU time.   */
/*   The scope distinguishes one revolution, all complete            */
/*   revolutions, partial data, and all samples.                       */
/*                                                                    */
/* Input:                                                             */
/*   scope - REV, PARTIAL, FULL_REVS, or ALL_SAMPLES                   */
/*   revolution - completed revolution count                           */
/*   algorithm - standard or simplified                                */
/*   summary - aggregate cycles and time                               */
/*                                                                    */
/* Output: int - communication or formatting status                   */
/* ------------------------------------------------------------------ */
int pedal_telemetry_timing(const char *scope, uint32_t revolution,
                           const char *algorithm,
                           const PedalTimingSummary *summary)
{
    if (scope == NULL || algorithm == NULL || summary == NULL) {
        return -EINVAL;
    }
    return send_formatted(PEDAL_MESSAGE_PLAIN,
        "TIMING %s=%u algorithm=%s samples=%u cycles=%llu time=%.2f [us] mean=%.2f [cycles/sample] %.3f [us/sample] max=%u [cycles]",
        scope, (unsigned int)revolution, algorithm, (unsigned int)summary->calls,
        (unsigned long long)summary->total_cycles,
        summary->total_us,
        summary->average_cycles, summary->average_us,
        (unsigned int)summary->max_cycles);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_telemetry_timing_pair(const char *scope,        */
/*                                         uint32_t revolution,        */
/*                                         const PedalTimingStats      */
/*                                         *standard,                  */
/*                                         const PedalTimingStats      */
/*                                         *simplified)                */
/*                                                                    */
/* Description:                                                       */
/*   Converts cycle accumulators to time and reports both algorithms  */
/*   for the same replay segment. Stops if formatting or queuing fails. */
/*                                                                    */
/* Input:                                                             */
/*   scope - segment name                                              */
/*   revolution - number of completed revolutions                     */
/*   standard - standard algorithm cycle accumulator                   */
/*   simplified - simplified algorithm cycle accumulator               */
/*                                                                    */
/* Output:                                                            */
/*   int - 0 or a negative timing, formatting, or communication code  */
/* ------------------------------------------------------------------ */
int pedal_telemetry_timing_pair(const char *scope, uint32_t revolution,
                                const PedalTimingStats *standard,
                                const PedalTimingStats *simplified)
{
    PedalTimingSummary summary;
    int rc = pedal_timing_summarize(standard, &summary);
    if (rc < 0) {
        return rc;
    }
    rc = pedal_telemetry_timing(scope, revolution, "standard", &summary);
    if (rc < 0) {
        return rc;
    }
    rc = pedal_timing_summarize(simplified, &summary);
    if (rc < 0) {
        return rc;
    }
    return pedal_telemetry_timing(scope, revolution, "simplified", &summary);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_telemetry_pacing(uint32_t late_samples,         */
/*                                    uint32_t total_samples)          */
/*                                                                    */
/* Description:                                                       */
/*   Reports the requested 200 Hz rate and any observed deadline      */
/*   misses. A late sample still gets processed; it is not discarded. */
/*                                                                    */
/* Input:                                                             */
/*   late_samples - number handled after their scheduled deadline     */
/*   total_samples - number of samples in the replay                  */
/*                                                                    */
/* Output: int - communication or formatting status                   */
/* ------------------------------------------------------------------ */
int pedal_telemetry_pacing(uint32_t late_samples, uint32_t total_samples)
{
    return send_formatted(late_samples == 0u ? PEDAL_MESSAGE_PLAIN :
                          PEDAL_MESSAGE_WARNING,
                          "PACING target=%u [Hz] samples=%u late=%u",
                          (unsigned int)PEDAL_SAMPLE_RATE_HZ,
                          (unsigned int)total_samples,
                          (unsigned int)late_samples);
}

/* ------------------------------------------------------------------ */
/* Creates: static int report_algorithm(const char *name,            */
/*                                      uint32_t revolutions,          */
/*                                      const PedalTimingStats *all,   */
/*                                      const PedalTimingStats *full)  */
/*                                                                    */
/* Description:                                                       */
/*   Reports average computation cost per processed sample and per    */
/*   complete revolution. The revolution mean excludes any partial   */
/*   final turn. Time is derived from measured CPU cycles.            */
/*                                                                    */
/* Input:                                                             */
/*   name - algorithm label                                            */
/*   revolutions - number of complete revolutions                     */
/*   all - timings for all processed samples                          */
/*   full - timings within complete revolutions                       */
/*                                                                    */
/* Output: int - 0 or a negative timing or communication code        */
/* ------------------------------------------------------------------ */
static int report_algorithm(const char *name, uint32_t revolutions,
                            const PedalTimingStats *all,
                            const PedalTimingStats *full)
{
    PedalTimingSummary per_sample;
    PedalTimingSummary full_summary;
    int rc = pedal_timing_summarize(all, &per_sample);
    if (rc < 0) {
        return rc;
    }
    rc = pedal_timing_summarize(full, &full_summary);
    if (rc < 0 || revolutions == 0u) {
        return rc < 0 ? rc : -ENODATA;
    }
    rc = send_formatted(PEDAL_MESSAGE_PLAIN, "%s", name);
    if (rc < 0) {
        return rc;
    }
    rc = send_formatted(PEDAL_MESSAGE_PLAIN,
                        "  average per sample: %.2f [cycles], %.3f [us]",
                        per_sample.average_cycles, per_sample.average_us);
    if (rc < 0) {
        return rc;
    }
    return send_formatted(PEDAL_MESSAGE_PLAIN,
                          "  average per revolution: %.2f [cycles], %.3f [us]",
                          (double)full_summary.total_cycles / revolutions,
                          full_summary.total_us / revolutions);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_telemetry_final(uint32_t samples,               */
/*                                    uint32_t revolutions,           */
/*                                    const PedalTimingStats *... )   */
/*                                                                    */
/* Description:                                                       */
/*   Sends only the requested replay counts and computation averages */
/*   after all samples have been processed.                           */
/*                                                                    */
/* Input:                                                             */
/*   samples - number of processed samples                            */
/*   revolutions - number of complete revolutions                     */
/*   standard_all - standard timing across all samples               */
/*   standard_revolutions - standard timing in complete turns         */
/*   simplified_all - simplified timing across all samples           */
/*   simplified_revolutions - simplified timing in complete turns     */
/*                                                                    */
/* Output: int - 0 or negative timing or communication code          */
/* ------------------------------------------------------------------ */
int pedal_telemetry_final(uint32_t samples, uint32_t revolutions,
                          const PedalTimingStats *standard_all,
                          const PedalTimingStats *standard_revolutions,
                          const PedalTimingStats *simplified_all,
                          const PedalTimingStats *simplified_revolutions)
{
    if (standard_all == NULL || standard_revolutions == NULL ||
        simplified_all == NULL || simplified_revolutions == NULL) {
        return -EINVAL;
    }
    int rc = send_formatted(PEDAL_MESSAGE_PLAIN,
                            "samples=%u sampling_frequency=%u [Hz] revolutions=%u",
                            (unsigned int)samples,
                            (unsigned int)PEDAL_SAMPLE_RATE_HZ,
                            (unsigned int)revolutions);
    if (rc < 0) {
        return rc;
    }
    rc = report_algorithm("standard", revolutions, standard_all,
                          standard_revolutions);
    if (rc < 0) {
        return rc;
    }
    return report_algorithm("simplified", revolutions, simplified_all,
                            simplified_revolutions);
}
