---
icon: chart-line
---

# Exponential Profiles

## What is a Motion Profile?

A motion profile is a trajectory that defines how a mechanism should move from one position to another over time. Instead of commanding a mechanism to instantly jump to a target position (which would require infinite acceleration), motion profiles generate a smooth path that respects the physical limitations of your system.

Motion profiles output a series of intermediate setpoints (position and velocity) that your closed-loop controller tracks. This results in:

* **Predictable motion** - You know exactly how long a movement will take
* **Smooth transitions** - No jerky movements that stress mechanical components
* **Controlled acceleration** - Prevents excessive current draw and brownouts

## Trapezoidal vs Exponential Profiles

### Trapezoidal Profiles

Trapezoidal profiles are the most common type. The name comes from the trapezoidal shape of the velocity-time graph.

```
Velocity
  ^
  |      ___________
  |     /           \
  |    /             \
  |   /               \
  |  /                 \
  |_/___________________\____> Time
```

**Key characteristics:**

* Constant acceleration until max velocity is reached
* Cruise at max velocity (if distance is long enough)
* Constant deceleration to stop at target
* Assumes unlimited torque during acceleration phases

**When trapezoidal profiles struggle:**

* When a disturbance knocks the mechanism off the profile
* When the motor cannot actually provide the commanded acceleration
* When current limiting reduces available torque

### Exponential Profiles

Exponential profiles model how DC motors actually behave. Instead of assuming constant acceleration, they account for the motor's torque-speed curve and the system's physical dynamics.

```
Velocity
  ^
  |           __________
  |         _/
  |       _/
  |     _/
  |   _/
  |__/____________________> Time
```

**Key characteristics:**

* Acceleration naturally decreases as velocity increases (following motor physics)
* Uses motor kV (velocity constant) and kA (acceleration constant) to model behavior
* More accurately represents what the motor can actually achieve
* Better disturbance rejection

{% hint style="info" %}
Exponential profiles are named for the exponential approach to maximum velocity, which mirrors the natural response of a first-order system like a DC motor.
{% endhint %}

## The Physics Behind Exponential Profiles

DC motors produce torque proportional to current. However, as the motor spins faster, back-EMF reduces the effective voltage across the windings, which reduces current and therefore torque.

The relationship is:

$$V = kS + kV \cdot \omega + kA \cdot \alpha$$

Where:

* **V** = Applied voltage
* **kS** = Static friction voltage (voltage to overcome stiction)
* **kV** = Velocity constant (volts per unit velocity)
* **kA** = Acceleration constant (volts per unit acceleration)
* **ω** = Angular velocity
* **α** = Angular acceleration

Rearranging for acceleration:

$$\alpha = \frac{V - kS - kV \cdot \omega}{kA}$$

This shows that as velocity (ω) increases, the available acceleration (α) decreases. This is the fundamental insight that exponential profiles capture.

## When to Use Each Profile Type

| Situation                               | Recommended Profile      | Why                                                                                                                          |
| --------------------------------------- | ------------------------ | ---------------------------------------------------------------------------------------------------------------------------- |
| Motor is always current-limited         | Trapezoidal              | Current limiting makes torque roughly constant regardless of speed                                                           |
| Motor is never current-limited          | Exponential              | Motor follows natural torque-speed curve                                                                                     |
| Sometimes current-limited               | Hybrid (not yet in YAMS) | See [this whitepaper](https://www.chiefdelphi.com/t/whitepaper-trapezoidal-exponential-motion-profiling/443468/12?u=nstrike) |
| Heavy mechanism with adequate motor     | Exponential              | Gravity/friction dominate, exponential handles disturbances better                                                           |
| Lightweight mechanism                   | Either                   | Both work well when inertia is low                                                                                           |
| Mechanism with significant disturbances | Exponential              | Better recovery when knocked off profile                                                                                     |
| Precise timing required                 | Trapezoidal              | Deterministic duration calculation                                                                                           |

{% hint style="success" %}
**Rule of thumb**: If your mechanism works well with trapezoidal profiles, keep using them. Switch to exponential profiles when you encounter disturbance rejection issues or when you want the profile to better match your motor's capabilities.
{% endhint %}

## Exponential Profiles in YAMS

YAMS provides `ExponentialProfilePIDController` which combines an exponential motion profile with a PID controller. This replaces `ProfiledPIDController` (which uses trapezoidal profiles) or `PIDController` (which has no profiling).

### Creating Constraints

YAMS provides factory methods to create exponential profile constraints for common mechanism types:

#### For Elevators

```java
ExponentialProfile.Constraints constraints = ExponentialProfilePIDController.createElevatorConstraints(
    Volts.of(12),        // Maximum voltage during profile
    DCMotor.getNEO(2),   // Motor type and quantity
    Pounds.of(15),       // Carriage mass
    Inches.of(1),        // Drum radius (or sprocket pitch radius)
    gearing              // MechanismGearing object
);
```

#### For Arms

```java
ExponentialProfile.Constraints constraints = ExponentialProfilePIDController.createArmConstraints(
    Volts.of(12),        // Maximum voltage during profile
    DCMotor.getKrakenX60(1), // Motor type and quantity
    Pounds.of(8),        // Arm mass (at center of mass)
    Inches.of(18),       // Arm length (to center of mass)
    gearing              // MechanismGearing object
);
```

#### For Flywheels

```java
ExponentialProfile.Constraints constraints = ExponentialProfilePIDController.createFlywheelConstraints(
    Volts.of(12),        // Maximum voltage during profile
    DCMotor.getFalcon500(1), // Motor type and quantity
    Pounds.of(2),        // Flywheel mass
    Inches.of(2),        // Flywheel radius
    gearing              // MechanismGearing object
);
```

#### Custom Constraints

If the factory methods don't fit your use case, you can create constraints directly:

```java
ExponentialProfile.Constraints constraints = ExponentialProfilePIDController.createConstraints(
    Volts.of(12),                    // Maximum voltage
    RotationsPerSecond.of(100),      // Maximum velocity (kV derived)
    RotationsPerSecondPerSecond.of(200) // Maximum acceleration (kA derived)
);
```

### Using with SmartMotorControllerConfig

```java
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(new ExponentialProfilePIDController(
        30,  // kP
        0,   // kI
        0,   // kD
        ExponentialProfilePIDController.createElevatorConstraints(
            Volts.of(12),
            DCMotor.getNEO(2),
            Pounds.of(15),
            Inches.of(1),
            gearing
        )
    ))
    .withGearing(gearing)
    .withFeedforward(new ElevatorFeedforward(0, 0.5, 0, 0))
    .withTelemetry("Elevator", TelemetryVerbosity.HIGH);
```

## Tuning Exponential Profiles

### Step 1: Characterize Your System

The exponential profile needs accurate kV and kA values. You have two options:

1. **Use the factory methods** - They calculate kV and kA from motor specs and mechanism parameters
2. **Run SysId** - Get experimental values from your actual mechanism

{% hint style="warning" %}
Factory methods use theoretical values. Real mechanisms have friction, cable drag, and other losses. SysId values are more accurate but require testing.
{% endhint %}

### Step 2: Set Maximum Voltage

The maximum voltage determines how aggressively the profile will command the mechanism. Consider:

* **12V** - Full battery voltage, most aggressive
* **10V** - Leaves headroom for PID corrections
* **8V** - Conservative, good for testing

### Step 3: Tune PID Gains

With exponential profiles, the feedforward does most of the work. The PID handles:

* **kP** - Corrects position error. Start low (1-5) and increase until responsive
* **kI** - Usually not needed with good feedforward. Use only if steady-state error persists
* **kD** - Dampens oscillations. Add if mechanism oscillates around target

### Step 4: Add Feedforward

Even with exponential profiles, you still need feedforward for:

* **Gravity compensation** (arms, elevators) - The profile doesn't know about gravity
* **Friction compensation** (kS) - Helps overcome stiction

```java
// For an arm
.withFeedforward(new ArmFeedforward(
    0.1,  // kS - voltage to overcome friction
    0.3,  // kG - voltage to hold horizontal
    0,    // kV - usually 0 since profile handles this
    0     // kA - usually 0 since profile handles this
))

// For an elevator
.withFeedforward(new ElevatorFeedforward(
    0.1,  // kS - voltage to overcome friction
    0.2,  // kG - voltage to counteract gravity
    0,    // kV - usually 0 since profile handles this
    0     // kA - usually 0 since profile handles this
))
```

{% hint style="info" %}
When using exponential profiles, the profile itself handles the velocity and acceleration feedforward. You typically only need kS (static friction) and kG (gravity) in your feedforward object.
{% endhint %}

## Motor Controller Support

| Motor Controller         | Trapezoidal Profile      | Exponential Profile           | Notes                       |
| ------------------------ | ------------------------ | ----------------------------- | --------------------------- |
| TalonFX (Kraken, Falcon) | On-device (Motion Magic) | On-device (Motion Magic Expo) | Best performance            |
| TalonFXS                 | On-device (Motion Magic) | On-device (Motion Magic Expo) | Best performance            |
| SparkMax/SparkFlex       | On-device (MAXMotion)    | RoboRIO                       | Exponential runs on RoboRIO |
| Nova                     | RoboRIO                  | RoboRIO                       | All profiles run on RoboRIO |

{% hint style="info" %}
When exponential profiles run on the RoboRIO, there's slightly more latency compared to on-device execution. However, the profile calculation is still very fast and suitable for most mechanisms.
{% endhint %}

## Common Issues and Solutions

### Profile is too slow

* Increase maximum voltage
* Check that your mechanism parameters (mass, length, gearing) are accurate
* Verify motor type matches actual hardware

### Profile overshoots target

* Decrease kP
* Add kD for damping
* Check that kV and kA values aren't underestimated

### Mechanism can't keep up with profile

* Your motor may be undersized for the mechanism
* Reduce maximum voltage
* Check for mechanical binding or excessive friction

### Profile works in sim but not on real robot

* Run SysId to get real kV and kA values
* Account for friction with kS in feedforward
* Check current limits aren't restricting torque

## Example: Complete Elevator with Exponential Profile

```java
public class ElevatorSubsystem extends SubsystemBase {

    private final Distance chainPitch = Inches.of(0.25);
    private final int toothCount = 22;
    private final Mass carriageMass = Pounds.of(15);
    private final DCMotor motors = DCMotor.getNEO(2);
    private final MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(5, 4));

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withDrumRadius(chainPitch, toothCount)
        .withClosedLoopController(new ExponentialProfilePIDController(
            15, 0, 0,
            ExponentialProfilePIDController.createElevatorConstraints(
                Volts.of(10), // Conservative voltage for testing
                motors,
                carriageMass,
                chainPitch.times(toothCount).div(2 * Math.PI),
                gearing
            )
        ))
        .withSimClosedLoopController(15, 0, 0, 
            MetersPerSecond.of(1), MetersPerSecondPerSecond.of(2))
        .withFeedforward(new ElevatorFeedforward(0.1, 0.25, 0, 0))
        .withSimFeedforward(new ElevatorFeedforward(0, 0.2, 0, 0))
        .withGearing(gearing)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH);

    private final SparkMax leftSpark = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax rightSpark = new SparkMax(2, MotorType.kBrushless);
    
    private final SmartMotorController motor = new SparkWrapper(
        leftSpark, 
        motors, 
        motorConfig.withFollowers(Pair.of(rightSpark, true))
    );

    private final Elevator elevator = new Elevator(
        new ElevatorConfig()
            .withSoftLimits(Meters.of(0), Meters.of(1.2))
            .withHardLimits(Meters.of(-0.05), Meters.of(1.3))
            .withStartingPosition(Meters.of(0))
            .withCarriageWeight(carriageMass)
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH),
        motor
    );

    public Command goToHeight(Distance height) {
        return elevator.setGoalCommand(height);
    }
}
```

## Further Reading

* [CTRE Motion Magic Expo Documentation](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo) - CTRE's explanation of exponential profiles
* [How do I use Exponential Profiles?](../../how-to/how-do-i-use-exponential-profiles.md) - Quick how-to guide
* [Trapezoidal/Exponential Motion Profiling Whitepaper](https://www.chiefdelphi.com/t/whitepaper-trapezoidal-exponential-motion-profiling/443468/12?u=nstrike) - Advanced hybrid profiling techniques
* [WPILib Tuning Guide](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-turret.html) - General PID and feedforward tuning
