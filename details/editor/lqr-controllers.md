---
icon: chart-line
---

# LQR Controllers

## What is LQR?

**Linear Quadratic Regulator (LQR)** is an optimal control algorithm that automatically computes controller gains to minimize a cost function balancing state error and control effort. Unlike manually-tuned PID controllers, LQR uses a mathematical model of your system to determine the optimal feedback gains.

{% hint style="info" %}
**Key Difference from PID**: With PID, you manually tune kP, kI, and kD through trial and error. With LQR, you specify *how much you care* about position error vs velocity error vs control effort, and the algorithm computes the optimal gains for you.
{% endhint %}

## Why Use LQR?

### Advantages

1. **Model-Based Tuning**: LQR uses your mechanism's physical model (mass, gearing, motor characteristics) to compute gains, reducing trial-and-error tuning
2. **Multi-State Control**: Naturally handles both position and velocity simultaneously
3. **Optimal Performance**: Mathematically minimizes a cost function you define
4. **Consistent Behavior**: Gains are derived from physics, making them more predictable across different setpoints

### When to Consider LQR

- **Complex mechanisms** where PID tuning is difficult
- **High-performance requirements** where optimal response matters
- **Educational settings** where understanding control theory is valuable
- **Mechanisms with well-characterized models** (accurate mass, inertia, motor constants)

### When PID May Be Better

- **Simple mechanisms** that tune easily with PID
- **Limited modeling data** (unknown mass, inertia, etc.)
- **Quick prototyping** where tuning time is limited
- **Mechanisms with significant nonlinearities** that LQR's linear model can't capture

## LQR Execution in YAMS

{% hint style="warning" %}
**Important**: LQR controllers are **not natively supported** by any motor controller hardware. When you configure an LQR controller in YAMS, the closed loop control **always runs on the RoboRIO** via `SmartMotorController.iterateClosedLoop()`.
{% endhint %}

### Why RoboRIO-Only?

Motor controller vendors (CTRE, REV, ThriftyBot) implement PID controllers on their hardware, but none support LQR natively. LQR requires:

1. Matrix operations (solving Riccati equations)
2. State-space model storage
3. Kalman filter state estimation
4. Multi-state feedback computation

These operations are beyond what current motor controller firmware supports.

### Performance Implications

| Aspect | Motor Controller PID | RoboRIO LQR |
|--------|---------------------|-------------|
| **Update Rate** | 1kHz+ (on controller) | 50Hz (robot loop) or custom period |
| **Latency** | Minimal (~1ms) | CAN bus round-trip (~5-20ms) |
| **CPU Load** | None on RoboRIO | Small increase on RoboRIO |
| **Best For** | Fast loops, simple control | Complex control, slower mechanisms |

{% hint style="info" %}
For most FRC mechanisms (arms, elevators), the 50Hz update rate is sufficient. LQR's optimal gains often compensate for the slower update rate compared to motor controller PID.
{% endhint %}

## Configuring LQR in YAMS

In YAMS, you create an `LQRConfig` to specify your mechanism type and tuning parameters, then create an `LQRController` from that config, and finally pass it to `SmartMotorControllerConfig.withClosedLoopController()`.

### LQRConfig Overview

`LQRConfig` requires three core parameters:
- **DCMotor**: The motor model (e.g., `DCMotor.getKrakenX60(1)`)
- **MechanismGearing**: Your gear reduction
- **MomentOfInertia**: The rotational inertia of your mechanism

Then you specify the mechanism type with one of:
- `.withFlyWheel()` - For velocity-controlled flywheels
- `.withArm()` - For position-controlled arms
- `.withElevator()` - For position-controlled elevators

### Flywheel LQR Configuration

Flywheels use a single-state model (velocity only):

```java
LQRConfig flywheelLQRConfig = new LQRConfig(
        DCMotor.getKrakenX60(1),                           // Motor
        new MechanismGearing(GearBox.fromReductionStages(2)), // Gearing
        KilogramSquareMeters.of(0.005)                     // Moment of inertia
    )
    .withFlyWheel(
        RadiansPerSecond.of(1.0),    // qelms: Velocity error tolerance (rad/s)
        RadiansPerSecond.of(0.01),   // modelTrust: Model standard deviation
        RadiansPerSecond.of(0.1)     // encoderTrust: Encoder standard deviation
    )
    .withControlEffort(Volts.of(12))  // R: Control effort tolerance
    .withMaxVoltage(Volts.of(12));    // Maximum output voltage

LQRController flywheelLQR = new LQRController(flywheelLQRConfig);
```

**Parameter Explanation**:
| Parameter | Unit | Description |
|-----------|------|-------------|
| `qelms` (velocity) | RadiansPerSecond | Velocity error tolerance. **Decrease** to penalize velocity errors more heavily (more aggressive). |
| `modelTrust` | RadiansPerSecond | Standard deviation of your model. **Lower** = trust the model more. |
| `encoderTrust` | RadiansPerSecond | Standard deviation of encoder readings. **Lower** = trust encoder more. |
| `controlEffort` | Volts | How much to penalize voltage usage. **Increase** for gentler control. |

### Arm LQR Configuration

Arms use a two-state model (position and velocity):

```java
LQRConfig armLQRConfig = new LQRConfig(
        DCMotor.getKrakenX60(1),
        new MechanismGearing(GearBox.fromReductionStages(100)),
        KilogramSquareMeters.of(0.5)  // Arm moment of inertia
    )
    .withArm(
        Radians.of(0.1),              // qelmsPosition: Position error tolerance
        RadiansPerSecond.of(1.0),     // qelmsVelocity: Velocity error tolerance
        Radians.of(0.01),             // modelPositionTrust: Model position std dev
        RadiansPerSecond.of(0.1),     // modelVelocityTrust: Model velocity std dev
        Radians.of(0.01)              // encoderPositionTrust: Encoder position std dev
    )
    .withControlEffort(Volts.of(12))
    .withMaxVoltage(Volts.of(12));

LQRController armLQR = new LQRController(armLQRConfig);
```

**Parameter Explanation**:
| Parameter | Unit | Description |
|-----------|------|-------------|
| `qelmsPosition` | Radians | Position error tolerance. **Decrease** for faster position correction. |
| `qelmsVelocity` | RadiansPerSecond | Velocity error tolerance. **Decrease** for smoother velocity tracking. |
| `modelPositionTrust` | Radians | Model position uncertainty. |
| `modelVelocityTrust` | RadiansPerSecond | Model velocity uncertainty. |
| `encoderPositionTrust` | Radians | Encoder measurement uncertainty. |

### Elevator LQR Configuration

Elevators use a two-state linear model (position and velocity in meters):

```java
LQRConfig elevatorLQRConfig = new LQRConfig(
        DCMotor.getKrakenX60(2),                              // 2 motors
        new MechanismGearing(GearBox.fromReductionStages(10)), // 10:1 reduction
        KilogramSquareMeters.of(0.01)                         // MOI (used for plant creation)
    )
    .withElevator(
        Meters.of(0.02),              // qelmsPosition: Position error tolerance (m)
        MetersPerSecond.of(0.5),      // qelmsVelocity: Velocity error tolerance (m/s)
        Meters.of(0.005),             // modelPositionTrust: Model position std dev
        MetersPerSecond.of(0.1),      // modelVelocityTrust: Model velocity std dev
        Meters.of(0.001),             // encoderPositionTrust: Encoder position std dev
        Kilograms.of(5),              // mass: Carriage mass
        Inches.of(1)                  // drumRadius: Spool/drum radius
    )
    .withControlEffort(Volts.of(12))
    .withMaxVoltage(Volts.of(12));

LQRController elevatorLQR = new LQRController(elevatorLQRConfig);
```

**Elevator-Specific Parameters**:
| Parameter | Unit | Description |
|-----------|------|-------------|
| `mass` | Kilograms | Total mass of the elevator carriage and load. |
| `drumRadius` | Meters/Inches | Radius of the spool or drum that the belt/rope wraps around. |

## Using LQRController with SmartMotorControllerConfig

Once you have an `LQRController`, pass it to `withClosedLoopController()`:

```java
// Create the LQR config and controller
LQRConfig lqrConfig = new LQRConfig(
        DCMotor.getKrakenX60(1),
        new MechanismGearing(GearBox.fromReductionStages(100)),
        KilogramSquareMeters.of(0.5)
    )
    .withArm(
        Radians.of(0.1),
        RadiansPerSecond.of(1.0),
        Radians.of(0.01),
        RadiansPerSecond.of(0.1),
        Radians.of(0.01)
    )
    .withControlEffort(Volts.of(12));

LQRController lqrController = new LQRController(lqrConfig);

// Pass to SmartMotorControllerConfig
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(100)))
    .withMomentOfInertia(Inches.of(24), Pounds.of(12))
    .withClosedLoopController(lqrController)  // Pass the LQRController here
    .withFeedforward(new ArmFeedforward(0.1, 0.45, 1.2, 0.05))
    .withIdleMode(MotorMode.BRAKE)
    .withTelemetry("LQRArm", TelemetryVerbosity.HIGH);
```

## Advanced LQR Options

### Aggressiveness

Use `withAggressiveness()` to scale overall controller response:

```java
LQRConfig config = new LQRConfig(motor, gearing, moi)
    .withArm(...)
    .withAggressiveness(10.0);  // Higher = more aggressive, typical range 1-20
```

### Measurement Delay Compensation

If your sensors have significant latency, compensate with:

```java
LQRConfig config = new LQRConfig(motor, gearing, moi)
    .withArm(...)
    .withMeasurementDelay(Milliseconds.of(25));  // Compensate for 25ms delay
```

### Custom Loop Period

By default, LQR uses a 20ms (50Hz) loop period. Override if using a faster loop:

```java
// LQRConfig uses the period internally for discrete-time calculations
// The period should match your actual control loop rate
```

## Simulation-Only LQR

You can use different controllers for simulation and real robot:

```java
// Create simulation LQR
LQRConfig simLqrConfig = new LQRConfig(DCMotor.getKrakenX60(1), gearing, moi)
    .withArm(Radians.of(0.1), RadiansPerSecond.of(1.0), 
             Radians.of(0.01), RadiansPerSecond.of(0.1), Radians.of(0.01))
    .withControlEffort(Volts.of(12));

LQRController simLqr = new LQRController(simLqrConfig);

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withGearing(gearing)
    .withMomentOfInertia(Inches.of(24), Pounds.of(12))
    // Real robot uses PID (runs on motor controller)
    .withClosedLoopController(5.0, 0, 0.5)
    // Simulation uses LQR for validation
    .withSimClosedLoopController(simLqr)
    .withIdleMode(MotorMode.BRAKE);
```

This is useful for:
- Validating LQR tuning in simulation before deploying
- Comparing LQR vs PID performance
- Using LQR's model-based approach to inform PID tuning

## Complete Example: LQR Arm

```java
public class LQRArmSubsystem extends SubsystemBase {
    private final SmartMotorController motor;
    private final Arm arm;

    public LQRArmSubsystem() {
        TalonFX talonFX = new TalonFX(1);
        
        // Define mechanism parameters
        DCMotor dcMotor = DCMotor.getKrakenX60(1);
        MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(100));
        MomentOfInertia moi = KilogramSquareMeters.of(0.5);
        
        // Create LQR configuration
        LQRConfig lqrConfig = new LQRConfig(dcMotor, gearing, moi)
            .withArm(
                Radians.of(0.05),             // Tight position tolerance
                RadiansPerSecond.of(0.5),     // Moderate velocity tolerance
                Radians.of(0.01),             // Trust the model
                RadiansPerSecond.of(0.1),
                Radians.of(0.005)             // Trust the encoder
            )
            .withControlEffort(Volts.of(10)) // Slightly aggressive
            .withMaxVoltage(Volts.of(12));
        
        // Create the LQR controller
        LQRController lqrController = new LQRController(lqrConfig);
        
        // Create motor controller config
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(gearing)
            .withMomentOfInertia(Inches.of(24), Pounds.of(12))
            .withClosedLoopController(lqrController)
            .withFeedforward(new ArmFeedforward(0.1, 0.45, 1.2, 0.05))
            .withAngleSoftLimits(Degrees.of(-10), Degrees.of(110))
            .withClosedLoopTolerance(Degrees.of(1))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry("LQRArm", TelemetryVerbosity.HIGH);
        
        motor = new TalonFXWrapper(talonFX, dcMotor, config);

        ArmConfig armConfig = new ArmConfig()
            .withTelemetry("LQRArm", TelemetryVerbosity.HIGH);

        arm = new Arm(armConfig, motor);
    }
    
    public Command goToAngle(Angle target) {
        return arm.goTo(() -> target);
    }
}
```

{% hint style="danger" %}
**Critical**: LQR runs on the RoboRIO via `SmartMotorController.iterateClosedLoop()`. If you use the `Arm`, `Elevator`, or other mechanism classes, this is handled automatically. If you use `SmartMotorController` directly, you must call `iterateClosedLoop()` in your periodic method.
{% endhint %}

## Tuning Strategy

### Starting Point

1. **Begin with conservative settings**:
   - Position qelms: `Radians.of(0.1)` or `Meters.of(0.05)`
   - Velocity qelms: `RadiansPerSecond.of(1.0)` or `MetersPerSecond.of(0.5)`
   - Control effort: `Volts.of(12)` (full battery = least aggressive)

2. **Trust values** (standard deviations):
   - Model trust: Start with small values (0.01) if your model is accurate
   - Encoder trust: Use values based on your encoder resolution and noise

### Increasing Responsiveness

- **Decrease** position qelms (e.g., `Radians.of(0.1)` → `Radians.of(0.05)`)
- **Decrease** control effort (e.g., `Volts.of(12)` → `Volts.of(8)`)

### Reducing Oscillation

- **Increase** control effort (e.g., `Volts.of(8)` → `Volts.of(12)`)
- **Decrease** velocity qelms to penalize velocity errors more

### Always Test in Simulation First

LQR can behave very differently with small parameter changes. Always validate in simulation before testing on the real robot.

## Troubleshooting LQR

### Oscillation

**Symptoms**: Mechanism oscillates around setpoint
**Solutions**:
- Increase control effort (R) to reduce aggressiveness
- Decrease velocity qelms to penalize velocity errors
- Add or increase feedforward kD term
- Check for mechanical issues (backlash, friction)

### Slow Response

**Symptoms**: Mechanism responds too slowly to setpoint changes
**Solutions**:
- Decrease control effort (R)
- Decrease position qelms to penalize position errors more
- Verify motion profile constraints aren't too conservative

### Steady-State Error

**Symptoms**: Mechanism doesn't quite reach setpoint
**Solutions**:
- Verify feedforward is properly tuned (especially kG for arms/elevators)
- Check that closed loop tolerance isn't too large
- Ensure model parameters (mass, inertia, gearing) are accurate

### Model Mismatch

**Symptoms**: Simulation works but real robot doesn't
**Solutions**:
- Re-measure mass and moment of inertia
- Verify gearing ratio is correct
- Run SysId to get accurate motor characterization
- Account for friction and other losses in feedforward

## LQR vs PID Quick Reference

| Aspect | PID | LQR |
|--------|-----|-----|
| **Tuning Method** | Trial and error | Model-based computation |
| **Parameters** | kP, kI, kD | Q matrix (qelms), R matrix (control effort) |
| **States Controlled** | Typically one (position or velocity) | Multiple (position AND velocity) |
| **Model Required** | No | Yes (motor, gearing, inertia) |
| **Optimality** | Not guaranteed | Mathematically optimal |
| **Hardware Support** | Motor controllers | RoboRIO only |
| **Update Rate** | 1kHz+ (on controller) | 50Hz (RoboRIO) |
| **YAMS API** | `withClosedLoopController(kP, kI, kD)` | `withClosedLoopController(LQRController)` |
