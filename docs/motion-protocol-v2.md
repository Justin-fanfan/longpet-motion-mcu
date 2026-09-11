# LongPet Motion Protocol V2

## 1. Scope

This document describes the ASCII command protocol between the LongPet host
and the ESP32-S3 Motion Controller. The controller owns the head Servo, the
four-wheel mecanum chassis, encoder PID, IMU heading hold, and the safety
watchdog. It does not implement the LongPet network API or the FOLLOW vision
algorithm.

UART settings:

```text
115200 baud, 8 data bits, no parity, 1 stop bit, no flow control
ESP32-S3 Serial1 RX = GPIO6, TX = GPIO7
```

Each command is one ASCII line terminated by `CR`, `LF`, or `CRLF`. The
receiver uses a fixed 64-byte buffer, accepts partial input and multiple
queued lines, and consumes at most 32 bytes per loop. Only a complete,
well-formed command that is legal in the current mode counts as valid link
traffic.

The parser accepts spaces between tokens and at the end of a line. Tabs,
other control bytes, non-ASCII bytes, extra fields, unknown commands, integer
overflow, and out-of-range values reject the whole line. A rejected line does
not refresh any link, motion, or target timeout.

## 2. Control modes

| Mode | Purpose | Chassis behavior |
|---|---|---|
| `SAFE` | Default and explicit safe state | Always stopped; no automatic head movement |
| `HEAD_ONLY` | Vision head-tracking test | `TARGET` may move the head; wheels are hard-blocked |
| `MANUAL` | Family remote control | `HEAD` and refreshed `MOVE` commands operate independently |
| `FOLLOW` | Future autonomous following | `TARGET` and head tracking are wired; chassis following is disabled until bbox-area calibration |

The initial mode after reset is `SAFE`. Any actual mode change stops the
chassis first and clears the active motion, manual motion lease, target
freshness, angle-turn state, and other transient movement state. A mode change
does not clear a latched fault; reset is still required for that.

## 3. Commands

### 3.1 Mode, stop, and diagnostics

```text
MODE SAFE
MODE HEAD_ONLY
MODE MANUAL
MODE FOLLOW

PING
STOP
STATUS
```

`MODE` is accepted in every mode and always leaves the selected mode stopped.
`STOP` immediately disables both TB6612 STBY groups, clears the four motor PWM
outputs and the active chassis command, and leaves the Servo at its current
position. It does not force the Servo to center and does not itself clear a
latched fault.

`PING` proves that the UART link and host are alive. It refreshes only the
general link timestamp. It never refreshes a MANUAL `MOVE` lease or a `TARGET`
freshness timestamp, and therefore cannot keep an old movement running.

`STATUS` prints the current mode, motion, stop reason, fault latch, target
availability, Servo pulse, and IMU readiness to the USB debug `Serial`. It is
not a response on `Serial1`.

### 3.2 Vision target

```text
TARGET <dx> <dy> <area>
```

Ranges are inclusive:

| Parameter | Range | Meaning |
|---|---:|---|
| `dx` | `-4096..4096` | Horizontal image error |
| `dy` | `-4096..4096` | Vertical image error; stored for future use |
| `area` | `0..16777216` | Detector bbox/area value |

`TARGET` is legal only in `HEAD_ONLY` and `FOLLOW`. `area == 0` means target
lost: `targetAvailable` becomes false and the chassis is stopped. A nonzero
target updates its own freshness timestamp and may update the head Servo.

In `HEAD_ONLY`, `TARGET` can never reach a wheel actuator. Large `dx`, a Servo
limit, and repeated target frames all remain head-only behavior. The current
head correction uses a 10-pixel deadband, a bounded correction of at most
40 microseconds per frame, and pulse clamping to 870..2270 microseconds. It
keeps the existing relation `servoPulseUs -= dx` (positive `dx` decreases the
pulse), but scales large errors instead of applying raw pixels as microseconds.

In `FOLLOW`, the same target/head path is available, but the old pixel-area
thresholds 5000/10000 do not drive the chassis. FOLLOW distance thresholds are
not calibrated yet for Tinyissimo V1.2 person bounding boxes.

### 3.3 Head commands

```text
HEAD LEFT [step]
HEAD RIGHT [step]
HEAD CENTER
```

These commands are legal only in `MANUAL`. `step` is an optional integer in
`1..100` microseconds; the default is 20. LEFT decreases the Servo pulse and
RIGHT increases it, matching the existing hardware direction convention.
Every accepted step changes the current target position once. Repeating the
command repeats the small position adjustment. When the host stops sending
HEAD commands, the Servo simply holds its last position; there is no head
watchdog and no automatic recentering. All positions are clamped to
`870..2270` microseconds, with `HEAD CENTER` at `1570` microseconds.

### 3.4 Manual chassis commands

```text
MOVE FORWARD <speed>
MOVE BACKWARD <speed>
MOVE ROTATE_LEFT <speed>
MOVE ROTATE_RIGHT <speed>
MOVE SHIFT_LEFT <speed>
MOVE SHIFT_RIGHT <speed>
```

These commands are legal only in `MANUAL`. `speed` is an integer in
`1..100` (`kMaximumSpeedCommand`). An invalid speed rejects the complete line;
the firmware does not silently clamp a bad protocol value.

Each accepted `MOVE` replaces the current manual chassis command and refreshes
`lastManualMotionCommandMs`. It is a lease, not a latch: the host must keep
refreshing it. The four motor outputs use the existing encoder PID and wheel
mapping. `ROTATE_LEFT` and `ROTATE_RIGHT` are continuous in-place rotations;
they have no target angle and run only while fresh `MOVE` commands arrive.

`LeftTurn(speed, angle)` and `RightTurn(speed, angle)` remain in the `Run`
layer for future autonomous angle turns, but V2 manual rotation never calls
them and never stops after an implicit 90-degree target.

## 4. Watchdogs and timestamps

The firmware maintains three separate timestamps:

| Timestamp | Refreshed by | Expiry behavior |
|---|---|---|
| `lastValidLinkCommandMs` | Any accepted legal command, including `PING` | Active chassis stops with recoverable `LINK_TIMEOUT` after 500 ms with no valid command |
| `lastManualMotionCommandMs` | `MOVE` only in `MANUAL` | Chassis stops with recoverable `MANUAL_COMMAND_TIMEOUT` after 500 ms without a new `MOVE` |
| `lastTargetCommandMs` | `TARGET` only in `HEAD_ONLY`/`FOLLOW` | Target becomes unavailable and chassis stops with recoverable `TARGET_LOST` after 500 ms |

The motion-specific watchdogs take precedence over the general link watchdog.
For example, repeated `PING` commands cannot extend a stale MANUAL `MOVE`, and
repeated `PING` commands cannot extend an old target. A new accepted command
after a link timeout may restore link state; a new `MOVE` or fresh `TARGET` is
still required to resume its associated function.

The host should send repeated `MOVE` commands at 100..200 ms intervals and
send `STOP` immediately when a button is released. A 10 Hz sender provides
margin below the 500 ms lease.

## 5. Stop and fault semantics

### Recoverable stops

The following stop reasons do not set `faultLatched` and can recover after a
new legal command:

- `POWER_ON`
- `STOP_COMMAND`
- `TARGET_LOST`
- `LINK_TIMEOUT`
- `MODE_CHANGED`
- `MANUAL_COMMAND_TIMEOUT`
- head-only/follow hold states (`TARGET_TRACKING_ONLY`,
  `FOLLOW_CHASSIS_DISABLED`)

All of them call `Run::Stop()`: STBY is driven LOW, real motor LEDC channels
4..7 are written with zero, direction pins are cleared, encoder windows and
PID transient state are reset, and continuous-rotation state is ended.

### Latched faults

The following remain latched until ESP reset or power cycle:

- `IMU_INIT_FAILED`
- `IMU_RUNTIME_FAILED`
- `CONTROL_OVERRUN`
- `TURN_TIMEOUT`
- another explicitly identified severe control/hardware failure

While latched, `PING`, `STATUS`, and debug output remain available. `MOVE`,
`TARGET`, and active `HEAD` commands are rejected; no chassis or active Servo
movement can resume. A mode command may change the displayed mode, but never
clears the fault latch.

## 6. Legacy compatibility

The old three-integer line remains accepted as a compatibility spelling:

```text
<dx> <dy> <area>
```

It is interpreted exactly as `TARGET <dx> <dy> <area>`, and is legal only in
`HEAD_ONLY` or `FOLLOW`. It no longer starts automatic `FORWARD` motion, does
not select a hidden 90-degree turn when the Servo reaches a limit, and never
uses the old 5000/10000 area thresholds to drive the chassis.

## 7. LongPet family remote sending guide

The family client should enter `MANUAL` before enabling the controls:

```text
MODE MANUAL\r\n
```

While the user holds Forward, send the current command about every 100 ms:

```text
MOVE FORWARD 20\r\n
MOVE FORWARD 20\r\n
MOVE FORWARD 20\r\n
```

When the button is released, send:

```text
STOP\r\n
```

The same pattern applies to Backward, Shift, and continuous rotation. A
single `MOVE ROTATE_LEFT 20` is not a 90-degree command; it expires after the
manual lease unless refreshed.

Head buttons are position steps, not leases. While Head Left is held, repeat:

```text
HEAD LEFT 20\r\n
```

When released, stop sending that HEAD command. The Servo holds its current
position. Head and chassis streams are independent, so this is valid:

```text
MOVE FORWARD 20\r\n
HEAD RIGHT 20\r\n
```

The client may send `PING` for link diagnostics, but it must never use PING as
the only keepalive for an active movement. On network, UART, process, or host
failure, the absence of refreshed `MOVE` commands is the safety mechanism that
stops the chassis.

## 8. Examples by mode

Head-only vision:

```text
MODE HEAD_ONLY\r\n
TARGET -85 12 7400\r\n
TARGET -20 10 7300\r\n
TARGET 0 8 0\r\n
```

Manual concurrent control:

```text
MODE MANUAL\r\n
MOVE FORWARD 20\r\n
HEAD RIGHT 20\r\n
MOVE FORWARD 20\r\n
STOP\r\n
HEAD CENTER\r\n
```

FOLLOW placeholder:

```text
MODE FOLLOW\r\n
TARGET 15 0 7400\r\n
```

The last example may move the head but must not move the chassis until the
Tinyissimo V1.2 bbox area/distance thresholds are separately calibrated and a
future firmware change explicitly enables following.
