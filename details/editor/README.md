---
icon: gear
---

# Smart Motor Controllers

## A quick primer on Motors and Motor Controllers

{% embed url="https://youtu.be/0BfR_viZSmQ" %}

{% embed url="https://youtu.be/pIKvQX5p1PE" %}

The presentation from the video series continues onto PID, Feedforward, and Motion Profiles as well!

{% file src="../../.gitbook/assets/Motors.pdf" %}

## The difference between Rotor, Mechanism, and Measurement

*

    <figure><img src="../../.gitbook/assets/image (19).png" alt=""><figcaption></figcaption></figure>
* **Rotor** is the motor output shaft.
* **Mechanism** is the gearbox AND sprocket output shaft.
* **Measurement** is the distance that the **Mechanism** travelled via its [**Mechanism Circumference**](../../how-to/how-to-find-your-mechanism-circumference.md)**.**

{% hint style="info" %}
Cascading elevator **Mechanism** gearings should be divided by the stages in the Elevator!
{% endhint %}

An easy way to do this with YAMS is by using `GearBox.fromReductionStages`and `Sprocket.fromStages`!

## Previous Configurations

YAMS does its best to prevent simple mistakes from turning into big headaches, so it will reset your previous smart motor controller configuration from other runs by default. You can configure this behavior with

```java
SmartMotorController config = new SmartMotorControllerConfig() //...
                                .withResetPreviousConfig(false);
```

## Power optimization is crucial to prevent brownouts

Power optimization can be done with [supply, and stator current limits, ramp rates](limiting-power-consumption.md), and adequately geared gearboxes and motors.

## Tuning our closed loop controllers

{% hint style="info" %}
## Motion Profiles

The rule of thumb is:

1. Use trapezoid profile if your DC motor is always current-limited or you’re doing torque control
2. Use exponential profile if your DC motor is never current-limited
3. Use [the algorithm in this whitepaper](https://www.chiefdelphi.com/t/whitepaper-trapezoidal-exponential-motion-profiling/443468/12?u=nstrike) if you’re sometimes current-limited (not implemented in YAMS, yet)
{% endhint %}

All SmartMotorControllers in YAMS support motion profiled PID's and feedforwards so you should have no limitation on your ability to tune your mechanism! Keep in mind that you can set [separate PID and Feedforward values for simulation only](simulation-only-pid-+-feedforward.md) since simulation will never exactly match real life.

You can learn how to tune PID and feedforwards from WPILib here!

{% embed url="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-turret.html" %}

```java
SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(0, 0, 0))
  .withSimFeedforward(new ArmFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
```

## External encoders

At the moment YAMS will only accept external encoders of the same vendor as this is the only vendor-supported way of using them. It is possible for external encoders to not have the same gearing as the rotor to the mechanism or not be a 1:1 to the mechanism so we do provide an external encoder gearing configuration option!

{% hint style="danger" %}
Absolute encoders cannot go beyond 1 rotation or 360 degrees, even less with discontinuity points!

DO **NOT** USE AN ABSOLUTE ENCODER WITH A GEAR RATIO WITH AN INPUT GREATER THAN 1 UNLESS YOU KNOW WHAT YOU'RE DOING!
{% endhint %}

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withExternalEncoderInverted(true)
      .withExternalEncoderGearing(new MechanismGearing(GearBox.fromStages("1:2")))
      .withUseExternalFeedbackEncoder(true)
      .withExternalEncoderZeroOffset(Degrees.of(0))
      .withExternalEncoderDiscontinuityPoint(Degrees.of(180)); // Sets the range of the absolute encoder to [-180, 180]
```

## Follower motors

Follower motors are not always the same inversion as your main motor so we help by using `Pair`s which have the motor and the inversion relative to the master.

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(new SparkMax(4, MotorType.kBrushless), false))
```

### Loosely Coupled Follower Motors

Loosely coupled followers are `SmartMotorController`s that are forwarded the velocity and position setpoints from the main motor. Loosely coupled followers are useful for motors which are not directly connected to the same shaft and where one motor just outputting the same dutycycle/voltage/current of the main motor is not sufficient control and could break the mechanism. Like 2 linear actuators with a cross bar attached at the top.

{% hint style="danger" %}
Loosely coupled followers **DO NOT** forward Voltage, or DutyCycle requests.
{% endhint %}

<pre class="language-java"><code class="lang-java">SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this);
SmartMotorController       motor2      = new TalonFXWrapper(new TalonFX(1), DCMotor.getKrakenX60(1), motor2Config.clone());
SmartMotorController       motor1      = new SparkWrapper(new SparkMax(2, MotorType.kBrushless), DCMotor.getNeo(1), 
<strong>                                                                motorConfig.withLooselyCoupledFollowers(motor2));
</strong></code></pre>

## You don't have to rely on YAMS to configure your SMC

YAMS is not the only way to configure your SmartMotorController! You can always fetch the config object, if available, from `SmartMotorController` like shown below

```java
(SparkFlex)smartMotorController.getMotorController();
(SparkFlexConfig)smartMotorController.getMotorControllerConfig();
```

## Some Mechanism config options modify SMC configs

Mechanism options like `.withSoftLimit` often will call the `SmartMotorControllerConfig.withSoftLimit` and do nothing else. This is to provide a seamless configuration for relevant areas in different configs.

## Simulation without a mechanism

`SmartMotorController`s require `SmartMotorController.simIterate` to be called in `simulationPeriodic` of your subsystem to provide simulation IF your motor controller is not used by a mechanism. It also proper to include the `SmartMotorController.updateTelemetry` as well.

```java
  @Override
  public void periodic()
  {
    smartMotorController.updateTelemetry();
  }
  
  @Override
  public void simulationPeriodic()
  {
    smartMotorController.simIterate();
  }
```

## Passing a Vendor Config

`SmartMotorControllerConfig` has the ability to use a supplied vendor config as the base to overwrite with the `SmartMotorControllerConfig` options. This allows you to set vendor specific configurations easily while creating the `SmartMotorControllerConfig`.

<pre class="language-java"><code class="lang-java">TalonFX                    armMotor      = new TalonFX(1);
SmartMotorControllerConfig motorConfig   = new SmartMotorControllerConfig(this)
      .withClosedLoopController(1, 0, 0)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
<strong>      .withVendorConfig(new TalonFXConfiguration().withVoltage(new VoltageConfigs().withPeakReverseVoltage(0)))
</strong>      .withControlMode(ControlMode.CLOSED_LOOP);
SmartMotorController       motor         = new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), motorConfig);
</code></pre>
