# How do I use absolute encoders on my Arm?

Relative encoders (built into most NEOs and Falcons) lose their zero reference on every power cycle, so an arm must be homed before closed-loop control is accurate. Absolute encoders retain their position across power cycles and eliminate the homing step entirely, which is especially important for arms that park at a non-zero angle. YAMS accepts an absolute encoder offset that maps the encoder's native range to your arm's coordinate system.

Absolute encoders report the physical position of a mechanism within a single rotation and retain that reading across power cycles and reboots. On an Arm (or Pivot) mechanism this means the robot always knows where the arm is the moment it powers on — no homing routine, no encoder drift from brownouts or e-stops.

## Supported encoder types

Each `SmartMotorController` wrapper only accepts the encoders from its own vendor ecosystem. Passing the wrong encoder type will throw an exception at runtime.

| Wrapper | Supported absolute encoders |
|---|---|
| `SparkWrapper` (SparkMax / SparkFlex) | `SparkAbsoluteEncoder` (`armMotor.getAbsoluteEncoder()`) |
| `TalonFXWrapper` (TalonFX / Kraken) | `CANcoder`, `CANdi` |
| `TalonFXSWrapper` (TalonFXS / Minion) | `CANcoder`, `CANdi` |

{% stepper %}
{% step %}
### Check your inversion

Using the vendor hardware client (REV Hardware Client or Phoenix Tuner X), apply positive power to the motor and graph the absolute encoder position. The position should **increase** as the motor turns in the positive direction.

If it decreases, invert the encoder reading:

```java
.withExternalEncoderInverted(true)
```
{% endstep %}

{% step %}
### Check your gear ratio and mount

The absolute encoder must complete **at most one full rotation** across the entire range of motion of the arm. Confirm this in the hardware client by moving the arm through its full travel and watching the encoder position — it must never wrap from 1 rotation back to 0 more than once.

Use `.withExternalEncoderGearing(1.0)` if the encoder is mounted directly on the mechanism shaft. If the encoder is driven through a reduction, pass the encoder-to-mechanism ratio.

{% hint style="warning" %}
If the gear ratio causes the encoder to exceed 1 rotation during the arm's range of motion, the encoder position will wrap unpredictably and **cannot** be used as the primary PID feedback device. Either remount the encoder closer to the mechanism output shaft or change the reduction so the encoder never exceeds 1 rotation.
{% endhint %}
{% endstep %}

{% step %}
### Set the discontinuity point

An absolute encoder wraps at its **discontinuity point** — the angle at which the reading jumps from its maximum value back to zero (or vice versa). There are two supported ranges:

- `Rotations.of(1.0)` — range is **\[0, 1\]**; the encoder wraps at 1 rotation back to 0
- `Rotations.of(0.5)` — range is **\[-0.5, 0.5\]**; the encoder wraps at ±0.5 rotations

```java
.withExternalEncoderDiscontinuityPoint(Rotations.of(1.0))  // [0, 1] range
// or
.withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))  // [-0.5, 0.5] range
```

{% hint style="info" %}
`SparkWrapper` **requires** the discontinuity point to be set explicitly. For `TalonFXWrapper` and `TalonFXSWrapper` it is optional but recommended for clarity.
{% endhint %}

{% hint style="warning" %}
If the arm's physical range of motion crosses the discontinuity point, the position will jump mid-travel and break closed-loop control. If this happens, remove the encoder, rotate the bearing so the discontinuity point falls outside the arm's travel range, and remount.
{% endhint %}
{% endstep %}

{% step %}
### Find your zero offset

Move the arm to exactly horizontal (0° from horizontal, parallel to the ground). Read the raw encoder position in the hardware client — that value is your zero offset.

```java
.withExternalEncoderZeroOffset(Degrees.of(33.25))  // replace with your measured value
```

Negative offsets are automatically converted to their positive equivalent (1 rotation is added internally).
{% endstep %}

{% step %}
### Configure the simulation starting position

Simulation has no physical encoder, so the absolute encoder reading is unavailable in sim. Use `.withSimStartingPosition()` to tell the simulator where the arm starts. On a real robot this value is ignored — the true encoder reading is always used.

```java
.withSimStartingPosition(Degrees.of(0))  // arm starts horizontal in sim
```
{% endstep %}
{% endstepper %}

***

## Complete examples

### SparkMax + SparkAbsoluteEncoder

```java
private final SparkMax armMotor = new SparkMax(1, MotorType.kBrushless);

private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(4, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
    .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
    .withSoftLimits(Degrees.of(-30), Degrees.of(100))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withIdleMode(MotorMode.BRAKE)
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
    .withStatorCurrentLimit(Amps.of(40))
    .withExternalEncoder(armMotor.getAbsoluteEncoder())
    .withExternalEncoderInverted(false)
    .withExternalEncoderGearing(1.0)
    .withExternalEncoderZeroOffset(Degrees.of(33.25))
    .withExternalEncoderDiscontinuityPoint(Rotations.of(1.0))  // [0, 1] range — required for SparkWrapper
    .withUseExternalFeedbackEncoder(true)
    .withSimStartingPosition(Degrees.of(0));

private final SmartMotorController smc = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
```

### TalonFX + CANcoder

```java
private final TalonFX armMotor = new TalonFX(1);
private final CANcoder cancoder = new CANcoder(2);

private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(4, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
    .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
    .withSoftLimits(Degrees.of(-30), Degrees.of(100))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withIdleMode(MotorMode.BRAKE)
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
    .withStatorCurrentLimit(Amps.of(40))
    .withExternalEncoder(cancoder)
    .withExternalEncoderInverted(false)
    .withExternalEncoderGearing(1.0)
    .withExternalEncoderZeroOffset(Degrees.of(33.25))
    .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))  // [-0.5, 0.5] range
    .withUseExternalFeedbackEncoder(true)
    .withSimStartingPosition(Degrees.of(0));

private final SmartMotorController smc = new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), motorConfig);
```

### TalonFXS + CANdi

The `CANdi` connects to the TalonFXS via its PWM port and is treated the same as a CANcoder for configuration purposes.

```java
private final TalonFXS armMotor = new TalonFXS(1);
private final CANdi candi = new CANdi(2);

private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(4, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
    .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
    .withSoftLimits(Degrees.of(-30), Degrees.of(100))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withIdleMode(MotorMode.BRAKE)
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
    .withStatorCurrentLimit(Amps.of(40))
    .withExternalEncoder(candi)
    .withExternalEncoderInverted(false)
    .withExternalEncoderGearing(1.0)
    .withExternalEncoderZeroOffset(Degrees.of(33.25))
    .withUseExternalFeedbackEncoder(true)
    .withSimStartingPosition(Degrees.of(0));

private final SmartMotorController smc = new TalonFXSWrapper(armMotor, DCMotor.getKrakenX60(1), motorConfig);
```

***

## Using absolute encoders on a Pivot

The exact same external encoder setup described above works identically for `Pivot` mechanisms (turrets, wrists, and any other single-axis rotational mechanism). The configuration API and all five setup steps are unchanged — just substitute your `PivotConfig` and the appropriate wrapper.

## Code Reference

The exponential arm example demonstrates configuring an absolute encoder with an offset:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/exponential_arm/java/frc/robot/subsystems/ExponentiallyProfiledArmSubsystem.java" %}
