#ifndef HEAD_DIRECTION_H
#define HEAD_DIRECTION_H

#include "motion_config.h"

#include <stdint.h>

namespace HeadDirection {

// Returns a delta to add to the current Servo pulse. These helpers express
// protocol semantics (physical left/right), not electrical pulse direction.
constexpr int pulseDeltaForPhysicalLeft(int magnitudeUs) {
    return MotionConfig::kServoPhysicalLeftPulseSign * magnitudeUs;
}

constexpr int pulseDeltaForPhysicalRight(int magnitudeUs) {
    return -pulseDeltaForPhysicalLeft(magnitudeUs);
}

// Camera coordinates grow to the right. A negative dx means the person is on
// the image's left side, so the head must move physically left.
constexpr int pulseDeltaForTargetDx(int32_t dx, int correctionUs) {
    return dx < 0 ? pulseDeltaForPhysicalLeft(correctionUs)
                  : (dx > 0 ? pulseDeltaForPhysicalRight(correctionUs) : 0);
}

// LongPet semantic offset: negative means the head is physically left of the
// chassis, positive means physically right. This value is safe for high-level
// policy; clients never need to know the electrical pulse direction.
constexpr int physicalOffsetUsForPulse(int pulseUs) {
    return -(pulseUs - MotionConfig::kServoCenterUs)
        * MotionConfig::kServoPhysicalLeftPulseSign;
}

}  // namespace HeadDirection

#endif
