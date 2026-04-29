# ESP32_PID_Cascaded_v5 Changelog

## Overview

This version introduces a **cascaded PID controller** with MT6701 magnetic encoder support for position control, replacing the single-loop balance-only controller in v4.

## What's New

### MT6701 Magnetic Encoder Support

| Feature | v4 | v5 |
|---------|----|----|
| Wheel position sensing | None (estimated from output) | MT6701 14-bit encoder via I2C |
| Wheel velocity | Fake estimate: `0.9 * prev + 0.1 * output` | Real measurement with low-pass filter |
| Position tracking | Not available | Cumulative tick counting with wraparound handling |

**New I2C device:**
- Address: `0x06`
- Shares I2C bus with MPU6050 (`0x68`)
- Reads angle register at `0x03`

### Cascaded Control Architecture

**v4: Single Loop**
```
Setpoint (fixed) ──► [Balance PID @ 100Hz] ──► Motors
                            ▲
                        MPU6050
```

**v5: Cascaded Loops**
```
Position Setpoint ──► [Position PID @ 50Hz] ──► Angle Offset ──► [Balance PID @ 100Hz] ──► Motors
        ▲                                              ▲
   MT6701 Encoder                                  MPU6050
```

### New Outer Loop (Position PID)

Controls wheel position to keep the robot in place.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Kp_pos` | 0.5 | Position proportional gain |
| `Ki_pos` | 0.0 | Position integral gain |
| `Kd_pos` | 2.0 | Position derivative (velocity damping) |
| `maxAngleOffset` | 5.0° | Maximum lean angle from outer loop |
| `positionSetpoint` | 0 | Target position (stay in place) |

**Loop timing:**
- Outer loop: 50 Hz (20ms)
- Inner loop: 100 Hz (10ms) - unchanged

### Extended WiFi Tuning

**v4 Commands:**
| Keys | Parameter |
|------|-----------|
| `q/a` | Kp |
| `w/s` | Kd |
| `e/d` | Ki |
| `z/x` | Setpoint |

**v5 Commands (all v4 commands plus):**
| Keys | Parameter |
|------|-----------|
| `r/f` | Position Kp |
| `t/g` | Position Kd |
| `y/h` | Position Ki |
| `p` | Reset encoder position to 0 |
| `?` | Print all current gains |

### Serial Output

**v4 format:**
```
dt, angle, output
```

**v5 format:**
```
dt, angle, output, wheelPosition, wheelVelocity, angleOffset
```

## Code Changes Summary

| Component | Lines Added | Description |
|-----------|-------------|-------------|
| MT6701 definitions | ~15 | Address, registers, variables |
| `initMT6701()` | 8 | Initialize encoder |
| `readMT6701()` | 12 | Read 14-bit angle via I2C |
| `updateEncoderPosition()` | 15 | Handle wraparound, track cumulative position |
| `calculateVelocity()` | 5 | Compute filtered velocity |
| `computePositionPID()` | 25 | Outer loop controller |
| WiFi handler extensions | ~30 | New tuning commands |

## Tuning Guide

1. **Start with outer loop disabled:**
   ```
   Kp_pos = 0, Ki_pos = 0, Kd_pos = 0
   ```

2. **Verify balance works** (existing v4 gains should transfer)

3. **Add velocity damping first:**
   - Increase `Kd_pos` from 0 to ~2.0
   - Robot should resist being pushed

4. **Add position hold:**
   - Increase `Kp_pos` from 0 to ~0.5
   - Robot should return to starting position

5. **Fine-tune:**
   - If oscillating: reduce `Kp_pos` or increase `Kd_pos`
   - If drifting slowly: add small `Ki_pos` (~0.001)

## Hardware Wiring

MT6701 encoder added to existing setup:

| MT6701 Pin | ESP32 Pin |
|------------|-----------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 21 (shared with MPU6050) |
| SCL | GPIO 22 (shared with MPU6050) |
