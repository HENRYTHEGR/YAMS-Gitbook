---
icon: hand-fist
---

# Arms

## Arm Angles

In accordance with WPILib standards Arm's are at 0deg when they are parallel from the ground, this way the feedforward calculations can be done with the `cos` of the angle.

### Create the `ArmConfig`

At this point we should have a `SmartMotorController` and a subsystem for our `Arm` which we bound the `SmartMotorControllerConfig` to already.

```java
SparkMax                   armMotor    = new SparkMax(1, MotorType.kBrushless);
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(4, 0, 0)
      .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withControlMode(ControlMode.CLOSED_LOOP);
SmartMotorController smartMotorController = new SparkWrapper(armMotor,
                                                             DCMotor.getNEO(1),
                                                             motorConfig);
ArmConfig armCfg = new ArmConfig();
Arm arm = new Arm(armCfg, smartMotorController);
```

The `SmartMotorControllerConfig` already has the gear ratio's to calculate rotor rotations into the `Arm` rotations and is configured for Telemetry and power optimizations.

{% hint style="warning" %}
Remember that it is always a good idea to have an absolute encoder attached to the Arm! YAMS only accepts absolute encoders of the same type in the `SmartMotorControllerConfig` like below.&#x20;

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withExternalEncoderInverted(true)
      .withExternalGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withUseExternalFeedbackEncoder(true);
```

However if you use a different type you can set the position of the mechanism using `SmartMotorController.setEncoderPosition` with the Mechanism position calculated from that absolute encoder.
{% endhint %}

### Length

`ArmConfig.withLength` allows you to easily set the length of the arm for simulation purposes and calculate the Moment of Inertia for the Arm Simulation inside of YAMS.&#x20;

```java
ArmConfig armCfg = new ArmConfig()
     .withLength(Feet.of(3));
```

Any changes in the length will require retuning of the Closed Loop Controller in the `SmartMotorControllerConfig` because the Moment of Inertia has changed.

### Mass

The mass (and resulting Moment of Inertia) used for the Arm Simulation is now configured on `SmartMotorControllerConfig` via `.withMomentOfInertia(Distance length, Mass mass)`.

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
     .withMomentOfInertia(Feet.of(3), Pounds.of(1));
```

Any changes in the length or mass will require retuning of the Closed Loop Controller in the `SmartMotorControllerConfig` because the Moment of Inertia has changed.

### Hard Limits

`ArmConfig.withHardLimits` defines the points in which there are physical stops that hopefully won't break on the real robot. Imagine them as an immovable force. These limits are used as an immovable force in the Arm simulation.

```java
ArmConfig armCfg = new ArmConfig()
      .withHardLimits(Degrees.of(-100), Degrees.of(200));
```

### Starting Position

`SmartMotorControllerConfig.withStartingPosition` is the way to set the starting position for an Arm without an Absolute Encoder. This defines the starting position of your Arm in simulation and will seed the encoder saying the Arm is starting at this angle.&#x20;

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withExternalEncoderInverted(true)
      .withExternalGearing(gearing(gearbox(3, 4)))
      .withUseExternalFeedbackEncoder(true)
      .withStartingPosition(Degrees.of(0))
      .withSimStartingPosition(Degrees.of(0)); // Parallel to the ground
```

### Horizontal Zero

`Arm.withHorizantalZero` allows you to set the offset of the Arm encoder or Absolute encoder which will make it read 0 when horizantal AKA parallel to the ground.

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withExternalEncoderInverted(true)
      .withExternalGearing(gearing(gearbox(3, 4)))
      .withUseExternalFeedbackEncoder(true)
      .withExternalEncoderZeroOffset(Degrees.of(0));
```

## Code Reference

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/simple_arm/java/frc/robot/subsystems/ArmSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/exponential_arm/java/frc/robot/subsystems/ExponentiallyProfiledArmSubsystem.java" %}
