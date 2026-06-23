# How do I use Exponential Profiles?

## What are exponential profiles?

Exponential profiles are best described by CTRE, however with YAMS they are supported by all motor controllers.&#x20;

{% embed url="https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo" %}

## How can I use Exponential Profiles?

All `SmartMotorController`s support Exponential Profiles via the `ExponentialProfilePIDController` closed loop controller. You would just have to use `ExponentialProfilePIDController` instead of `ProfiledPIDController` or `PIDController`.

```java
private final Distance chainPitch = Inches.of(0.25);
private final int toothCount = 22;
private final Mass     weight = Pounds.of(16);
private final DCMotor  motors = DCMotor.getNEO(1);
private final MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(3, 4));

private final SmartMotorControllerConfig motorConfig        = new SmartMotorControllerConfig(this)
      .withDrumRadius(chainPitch, toothCount)
      .withClosedLoopController(30, 0, 0)
      .withExponentialProfile(
      ExponentialProfilePIDController.createElevatorConstraints(
                                     Volts.of(12), // Maximum voltage during profile
                                     motors, // Motors
                                     weight, // Carraige weight
                                     chainPitch.times(toothCount).div(2 * Math.PI), // Drum radius
                                     gearing))) // Gearing
...
```

There are multiple ways to create constraints, like [`createArmConstraints`](https://yet-another-software-suite.github.io/YAMS/javadocs/yams/math/ExponentialProfilePIDController.html#createArmConstraints\(edu.wpi.first.units.measure.Voltage,edu.wpi.first.math.system.plant.DCMotor,edu.wpi.first.units.measure.Mass,edu.wpi.first.units.measure.Distance,yams.gearing.MechanismGearing\))  or [`createFlyWheelConstraints`](https://yet-another-software-suite.github.io/YAMS/javadocs/yams/math/ExponentialProfilePIDController.html#createFlywheelConstraints\(edu.wpi.first.units.measure.Voltage,edu.wpi.first.math.system.plant.DCMotor,edu.wpi.first.units.measure.Mass,edu.wpi.first.units.measure.Distance,yams.gearing.MechanismGearing\))  or even [`createConstraints`](https://yet-another-software-suite.github.io/YAMS/javadocs/yams/math/ExponentialProfilePIDController.html#createConstraints\(edu.wpi.first.units.measure.Voltage,edu.wpi.first.units.measure.AngularVelocity,edu.wpi.first.units.measure.AngularAcceleration\)).&#x20;

## When should I use Exponential Profiles?

Exponential Profiles are easier to tune than Trapezoidal Profiles (from `ProfiledPIDController`) and are very good at overcoming mechanical failures to a point. It CANNOT speed up a mechanism that is massively under-powered and slow to move. Trapezoidal Profiles often fail miserably when a mechanism experiences disturbances along its movement, aside from fixing those disturbances an Exponential Profile is probably your best chance at getting it "working" quickly.

