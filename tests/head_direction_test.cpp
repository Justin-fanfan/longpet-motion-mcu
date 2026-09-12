#include "../xiao_che/head_direction.h"
#include "../xiao_che/follow_safety.h"

#include <cassert>

int main() {
    using namespace HeadDirection;

    assert(pulseDeltaForPhysicalLeft(20) == 20);
    assert(pulseDeltaForPhysicalRight(20) == -20);

    assert(pulseDeltaForTargetDx(-80, 12) ==
           pulseDeltaForPhysicalLeft(12));
    assert(pulseDeltaForTargetDx(80, 12) ==
           pulseDeltaForPhysicalRight(12));
    assert(pulseDeltaForTargetDx(0, 12) == 0);
    assert(physicalOffsetUsForPulse(MotionConfig::kServoCenterUs) == 0);
    assert(physicalOffsetUsForPulse(MotionConfig::kServoCenterUs + 230) ==
           -230);
    assert(physicalOffsetUsForPulse(MotionConfig::kServoCenterUs - 230) ==
           230);

    using namespace FollowSafety;
    assert(isAllowed(Command::Stop));
    assert(isAllowed(Command::Forward));
    assert(isAllowed(Command::RotateLeft));
    assert(isAllowed(Command::RotateRight));
    assert(!isAllowed(Command::Rejected));
    assert(refreshesLease(LeaseEvent::FollowMove));
    assert(!refreshesLease(LeaseEvent::Ping));
    assert(!refreshesLease(LeaseEvent::Status));
    assert(!refreshesLease(LeaseEvent::Target));
    assert(!leaseExpired(1499, 1000, 500));
    assert(leaseExpired(1500, 1000, 500));
    return 0;
}
