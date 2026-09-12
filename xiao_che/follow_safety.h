#ifndef FOLLOW_SAFETY_H
#define FOLLOW_SAFETY_H

#include <stdint.h>

namespace FollowSafety {

enum class Command : uint8_t {
    Stop,
    Forward,
    RotateLeft,
    RotateRight,
    Rejected
};

enum class LeaseEvent : uint8_t {
    FollowMove,
    Ping,
    Status,
    Target
};

constexpr bool isAllowed(Command command) {
    return command == Command::Stop || command == Command::Forward
        || command == Command::RotateLeft
        || command == Command::RotateRight;
}

constexpr bool refreshesLease(LeaseEvent event) {
    return event == LeaseEvent::FollowMove;
}

constexpr bool leaseExpired(uint32_t now, uint32_t lastRefresh,
                            uint32_t timeoutMs) {
    return static_cast<uint32_t>(now - lastRefresh) >= timeoutMs;
}

}  // namespace FollowSafety

#endif
