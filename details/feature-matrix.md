---
icon: table-cells
---

# Motor Controller Feature Matrix

This page provides a comprehensive comparison of supported `SmartMotorControllerConfig` features across all motor controller wrappers in YAMS.

## Supported Motor Controllers

| Wrapper | Vendor | Hardware | Package Location |
|---------|--------|----------|------------------|
| `TalonFXWrapper` | CTRE | TalonFX (Kraken X60, Falcon 500) | `yams.motorcontrollers.remote` |
| `TalonFXSWrapper` | CTRE | TalonFXS (controls NEO, NEO 550, Vortex, Minion) | `yams.motorcontrollers.remote` |
| `SparkWrapper` | REV | SparkMax, SparkFlex | `yams.motorcontrollers.local` |
| `NovaWrapper` | ThriftyBot | ThriftyNova | `yams.motorcontrollers.local` |

## Feature Support Matrix

### Legend

| Symbol | Meaning |
|--------|---------|
| ✅ | Fully Supported |
| ⚠️ | Partially Supported (see notes) |
| ❌ | Not Supported |
| 🔧 | Requires Vendor Config |

---

### Basic Configuration

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Motor Inversion | ✅ | ✅ | ✅ | ✅ |
| Encoder Inversion | ✅ | ✅ | ✅ | ❌ |
| Idle Mode (Brake/Coast) | ✅ | ✅ | ✅ | ✅ |
| Reset Previous Config | ✅ | ✅ | ✅ | ✅ |
| Vendor Config Passthrough | ✅ | ✅ | ✅ | ✅ |
| Starting Position | ✅ | ✅ | ✅ | ✅ |

### Current Limits

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Stator Current Limit | ✅ | ✅ | ✅ | ✅ |
| Supply Current Limit | ✅ | ✅ | ✅ | ✅ |

### Voltage Configuration

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Voltage Compensation | ✅ | ✅ | ✅ | ✅ |
| Closed Loop Max Voltage | ✅ | ✅ | ✅ | ✅ |

### Ramp Rates

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Open Loop Ramp Rate | ✅ | ✅ | ✅ | ⚠️ (uses closed loop value) |
| Closed Loop Ramp Rate | ✅ | ✅ | ✅ | ✅ |

{% hint style="warning" %}
**Nova Note**: ThriftyNova does not support separate open loop and closed loop ramp rates. The closed loop ramp rate value is used for both.
{% endhint %}

### Gearing

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Mechanism Gearing | ✅ | ✅ | ✅ | ✅ |
| External Encoder Gearing | ✅ | ✅ | ✅ | ✅ |
| Mechanism Circumference (Linear) | ✅ | ✅ | ✅ | ✅ |

### Closed Loop Control

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| PID Controller | ✅ | ✅ | ✅ | ✅ |
| Simulation PID Controller | ✅ | ✅ | ✅ | ✅ |
| LQR Controller | ✅ | ✅ | ✅ | ✅ |
| Simulation LQR Controller | ✅ | ✅ | ✅ | ✅ |
| Closed Loop Tolerance | ✅ | ✅ | ✅ | ✅ |
| Control Period Override | ✅ | ✅ | ✅ | ✅ |

### Motion Profiles

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Trapezoidal Profile | ✅ (Motion Magic) | ✅ (Motion Magic) | ✅ (MAXMotion) | ✅ (RoboRIO) |
| Exponential Profile | ✅ (Motion Magic Expo) | ✅ (Motion Magic Expo) | ✅ (RoboRIO) | ✅ (RoboRIO) |
| Simulation Trapezoidal Profile | ✅ | ✅ | ✅ | ✅ |
| Simulation Exponential Profile | ✅ | ✅ | ✅ | ✅ |
| Velocity Trapezoidal Profile | ✅ | ✅ | ✅ | ✅ |

{% hint style="info" %}
**Profile Execution Location**:
- **TalonFX/TalonFXS**: Profiles run on the motor controller (Motion Magic)
- **SparkMax/SparkFlex**: Trapezoidal profiles run on motor controller (MAXMotion), exponential profiles run on RoboRIO
- **Nova**: All profiles run on RoboRIO
{% endhint %}

### Feedforward

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| SimpleMotorFeedforward | ✅ | ✅ | ✅ | ✅ |
| ArmFeedforward | ✅ | ✅ | ✅ | ✅ |
| ElevatorFeedforward | ✅ | ✅ | ✅ | ✅ |
| Simulation Feedforward | ✅ | ✅ | ✅ | ✅ |

### Soft Limits

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Angle Soft Limits | ✅ | ✅ | ✅ | ✅ |
| Distance Soft Limits | ✅ | ✅ | ✅ | ✅ |
| Continuous Wrapping | ✅ | ✅ | ✅ | ⚠️ (RoboRIO PID only) |

### External Encoders

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| External Encoder Support | ✅ | ✅ | ✅ | ✅ |
| External Encoder Inversion | ✅ | ✅ | ✅ | ❌ |
| External Encoder Zero Offset | ✅ | ✅ | ✅ | ❌ |
| Use External for Feedback | ✅ | ✅ | ✅ | ✅ |
| Feedback Synchronization | ✅ | ✅ | ✅ | ❌ |

**Supported External Encoder Types:**

| Wrapper | Supported Encoders |
|---------|-------------------|
| TalonFX | CANcoder, CANdi (PWM1/PWM2) |
| TalonFXS | CANcoder, CANdi (PWM1/PWM2) |
| SparkMax/Flex | SparkAbsoluteEncoder (Through Bore, REV Hex) |
| Nova | ThriftyNova ExternalEncoder (ABS type) |

### Follower Motors

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Hardware Followers | ✅ | ✅ | ✅ | ✅ |
| Follower Inversion | ✅ | ✅ | ✅ | ✅ |
| Loosely Coupled Followers | ✅ | ✅ | ✅ | ✅ |

{% hint style="warning" %}
**Follower Compatibility**: Followers must be the same brand as the leader motor controller. You cannot mix vendors for hardware followers.
{% endhint %}

### Telemetry

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Telemetry Name | ✅ | ✅ | ✅ | ✅ |
| Telemetry Verbosity | ✅ | ✅ | ✅ | ✅ |
| Custom Telemetry Config | ✅ | ✅ | ✅ | ✅ |

### Simulation

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| DCMotorSim | ✅ | ✅ | ✅ | ✅ |
| Moment of Inertia | ✅ | ✅ | ✅ | ✅ |
| Custom Sim Supplier | ✅ | ✅ | ✅ | ✅ |
| External Encoder Sim | ✅ | ✅ | ✅ | ⚠️ |

### Safety Features

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| Temperature Cutoff | ✅ | ✅ | ✅ | ✅ |

### SysId Integration

| Feature | TalonFX | TalonFXS | SparkMax/Flex | Nova |
|---------|---------|----------|---------------|------|
| SysId Routine Support | ✅ (SignalLogger) | ✅ (SignalLogger) | ✅ | ✅ |
| Custom SysId Config | ✅ | ✅ | ✅ | ✅ |

---

## Motor Type Support by Wrapper

### TalonFXWrapper
- Kraken X60 / Kraken X60 FOC
- Falcon 500 / Falcon 500 FOC

### TalonFXSWrapper
- NEO (via JST connector)
- NEO 550 (via JST connector)
- NEO Vortex (via JST connector)
- Minion (via JST connector, with Advanced Hall Support)

### SparkWrapper
- NEO (SparkMax with brushless mode)
- NEO 550 (SparkMax with brushless mode)
- NEO Vortex (SparkFlex)
- Any brushless motor (SparkMax/SparkFlex)

### NovaWrapper
- NEO
- NEO 550
- NEO Vortex
- Any motor compatible with ThriftyNova

---

## Configuration Examples

### TalonFX with CANcoder

```java
TalonFX motor = new TalonFX(1);
CANcoder encoder = new CANcoder(2);

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(1, 0, 0)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5)))
    .withExternalEncoder(encoder)
    .withExternalEncoderZeroOffset(Degrees.of(90))
    .withUseExternalFeedbackEncoder(true)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), config);
```

### TalonFXS with NEO

```java
TalonFXS motor = new TalonFXS(1);

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(0.5, 0, 0)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
    .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40))
    .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new TalonFXSWrapper(motor, DCMotor.getNEO(1), config);
```

### SparkMax with Absolute Encoder

```java
SparkMax spark = new SparkMax(1, MotorType.kBrushless);

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(1, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(360))
    .withExternalEncoder(spark.getAbsoluteEncoder())
    .withExternalEncoderInverted(true)
    .withUseExternalFeedbackEncoder(true)
    .withExternalEncoderZeroOffset(Degrees.of(45))
    .withFeedforward(new ArmFeedforward(0, 0.5, 0, 0))
    .withIdleMode(MotorMode.BRAKE)
    .withTelemetry("WristMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new SparkWrapper(spark, DCMotor.getNEO(1), config);
```

### ThriftyNova Basic Setup

```java
ThriftyNova nova = new ThriftyNova(1);

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(0.5, 0, 0)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5)))
    .withFeedforward(new SimpleMotorFeedforward(0, 0.12, 0))
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(30))
    .withTelemetry("IntakeMotor", TelemetryVerbosity.MEDIUM);

SmartMotorController smc = new NovaWrapper(nova, DCMotor.getNEO(1), config);
```

---

## Known Limitations

### ThriftyNova (NovaWrapper)
- No separate open/closed loop ramp rates
- No encoder inversion support
- No external encoder inversion
- No zero offset for external encoders
- No feedback synchronization threshold
- Continuous wrapping only works with RoboRIO-side PID

### SparkMax/SparkFlex (SparkWrapper)
- Exponential profiles run on RoboRIO, not on motor controller
- External encoders must be REV-compatible (Through Bore, Hex)

### TalonFX/TalonFXS
- Requires Phoenix 6 library
- CANcoder/CANdi must be on same CAN bus for best performance
