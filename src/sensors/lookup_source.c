#include "sensors/sensors.h"

#include <errno.h>
#include "lookup/samples.h"

/* ------------------------------------------------------------------ */
/* Creates: void pedal_sensors_reset(PedalSensorSource *source)        */
/*                                                                    */
/* Description:                                                       */
/*   Rewinds the finite lookup replay to its first sample.            */
/*                                                                    */
/* Input:                                                             */
/*   source - source state to reset                                    */
/*                                                                    */
/* Output:                                                            */
/*   void - source is reset in place                                  */
/* ------------------------------------------------------------------ */
void pedal_sensors_reset(PedalSensorSource *source)
{
    if (source != NULL) {
        source->next_index = 0u;
    }
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_sensors_next(PedalSensorSource *source,          */
/*                                 PedalSensorSample *sample)          */
/*                                                                    */
/* Description:                                                       */
/*   Copies one stored sample and advances the source cursor.         */
/*   Reaching the final sample is a normal completion result.         */
/*                                                                    */
/* Input:                                                             */
/*   source - lookup cursor                                           */
/*   sample - reading destination                                     */
/*                                                                    */
/* Output:                                                            */
/*   int - 1 sample, 0 complete, or -EINVAL                           */
/* ------------------------------------------------------------------ */
int pedal_sensors_next(PedalSensorSource *source, PedalSensorSample *sample)
{
    if (source == NULL || sample == NULL) {
        return -EINVAL;
    }
    if (source->next_index >= pedal_lookup_count) {
        return 0;
    }
    *sample = pedal_lookup_samples[source->next_index++];
    return 1;
}
