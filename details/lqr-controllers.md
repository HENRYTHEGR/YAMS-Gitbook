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

## How LQR Works

### The State-Space Model

LQR operates on a state-space representation of your system:

```
x' = Ax + Bu
y  = Cx + Du
```

Where:
- **x** = state vector (typically [position, velocity])
- **u** = control input (voltage)
- **A** = system dynamics matrix
- **B** = input matrix
- **C** = output matrix
- **D** = feedthrough matrix

For a DC motor-driven mechanism, YAMS automatically constructs these matrices from:
- Motor characteristics (`DCMotor`)
- Gearing ratio
- Moment of inertia or mass
- Number of motors

### The Cost Function

LQR minimizes a cost function:

$$J = \int_0^\infty (x^T Q x + u^T R u) \, dt$$

Where:
- **Q** = state cost matrix (how much you penalize position and velocity errors)
- **R** = control cost matrix (how much you penalize control effort/voltage)

**Intuition**:
- **Higher Q values** = more aggressive error correction (faster response, more overshoot)
- **Higher R values** = gentler control (slower response, less overshoot, lower power consumption)

### The Solution

The LQR algorithm solves the algebraic Riccati equation to find the optimal gain matrix **K** such that:

```
u = -Kx
```

This gain matrix is computed once (not iteratively) and provides optimal feedback gains.

## LQR Execution in YAMS

{% hint style="warning" %}
**Important**: LQR controllers are **not natively supported** by any motor controller hardware. When you configure an LQR controller in YAMS, the closed loop control **always runs on the RoboRIO** via `SmartMotorController.iterateClosedLoop()`.
{% endhint %}

### Why RoboRIO-Only?

Motor controller vendors (CTRE, REV, ThriftyBot) implement PID controllers on their hardware, but none support LQR natively. LQR requires:

1. Matrix operations (solving Riccati equations)
2. State-space model storage
3. Multi-state feedback computation

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

### Basic LQR Configuration

```java
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(100))) // 100:1 reduction
    .withMomentOfInertia(Inches.of(18), Pounds.of(15)) // Arm length and mass
    .withLQRController(
        VecBuilder.fill(0.1, 1.0),  // Q: [position error cost, velocity error cost]
        VecBuilder.fill(12.0)       // R: [voltage cost]
    )
    .withIdleMode(MotorMode.BRAKE)
    .withTelemetry("LQRArmMotor", TelemetryVerbosity.HIGH);
```

### Understanding Q and R Matrices

#### Q Matrix (State Costs)

The Q matrix penalizes state errors. For a position-velocity state:

```java
VecBuilder.fill(qPosition, qVelocity)
```

| Parameter | Effect of Increasing |
|-----------|---------------------|
| `qPosition` | Faster position correction, more aggressive |
| `qVelocity` | Penalizes velocity error, smoother velocity tracking |

**Typical Starting Values**:
- Position: `0.1` to `1.0` (in mechanism units squared, e.g., radians^2)
- Velocity: `1.0` to `10.0` (in mechanism units/second squared)

#### R Matrix (Control Cost)

The R matrix penalizes control effort:

```java
VecBuilder.fill(rVoltage)
```

| Parameter | Effect of Increasing |
|-----------|---------------------|
| `rVoltage` | Gentler control, slower response, less power |

**Typical Starting Values**:
- Voltage: `12.0` (using nominal battery voltage as reference)

### Tuning Strategy

1. **Start Conservative**: Begin with high R (e.g., 12.0) and moderate Q values
2. **Increase Responsiveness**: Lower R or increase Q position cost
3. **Reduce Oscillation**: Increase R or increase Q velocity cost
4. **Validate in Sim**: Always test in simulation before the real robot

```java
// Conservative start
.withLQRController(VecBuilder.fill(0.1, 1.0), VecBuilder.fill(12.0))

// More aggressive
.withLQRController(VecBuilder.fill(1.0, 2.0), VecBuilder.fill(6.0))

// Very aggressive (use with caution)
.withLQRController(VecBuilder.fill(2.0, 5.0), VecBuilder.fill(3.0))
```

## LQR with Motion Profiles

LQR can be combined with motion profiles for smooth trajectory following:

```java
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(100)))
    .withMomentOfInertia(Inches.of(18), Pounds.of(15))
    .withLQRController(
        VecBuilder.fill(0.5, 2.0),
        VecBuilder.fill(12.0),
        DegreesPerSecond.of(180),           // Max velocity
        DegreesPerSecondPerSecond.of(360)   // Max acceleration
    )
    .withFeedforward(new ArmFeedforward(0, 0.5, 0, 0))
    .withIdleMode(MotorMode.BRAKE)
    .withTelemetry("ProfiledLQRArm", TelemetryVerbosity.HIGH);
```

The motion profile generates smooth setpoints, and LQR tracks those setpoints optimally.

## LQR with Feedforward

**Always combine LQR with appropriate feedforward** for best results:

```java
// For arms
.withLQRController(VecBuilder.fill(0.5, 2.0), VecBuilder.fill(12.0))
.withFeedforward(new ArmFeedforward(kS, kG, kV, kA))

// For elevators
.withLQRController(VecBuilder.fill(0.5, 2.0), VecBuilder.fill(12.0))
.withFeedforward(new ElevatorFeedforward(kS, kG, kV, kA))

// For flywheels (velocity control)
.withLQRController(VecBuilder.fill(1.0), VecBuilder.fill(12.0)) // Single state for velocity
.withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
```

{% hint style="success" %}
Feedforward handles the predictable physics (gravity, friction, inertia), while LQR handles the unpredictable disturbances and errors.
{% endhint %}

## Simulation-Only LQR

You can use different controllers for simulation and real robot:

```java
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(100)))
    .withMomentOfInertia(Inches.of(18), Pounds.of(15))
    // Real robot uses PID
    .withClosedLoopController(5.0, 0, 0.5)
    // Simulation uses LQR for validation
    .withSimLQRController(
        VecBuilder.fill(0.5, 2.0),
        VecBuilder.fill(12.0)
    )
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
        
        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(100)))
            .withMomentOfInertia(Inches.of(24), Pounds.of(12)) // 24" arm, 12 lbs
            .withLQRController(
                VecBuilder.fill(0.5, 2.0),  // Q matrix
                VecBuilder.fill(12.0),      // R matrix
                DegreesPerSecond.of(120),   // Max velocity
                DegreesPerSecondPerSecond.of(240) // Max acceleration
            )
            .withFeedforward(new ArmFeedforward(0.1, 0.45, 1.2, 0.05))
            .withAngleSoftLimits(Degrees.of(-10), Degrees.of(110))
            .withClosedLoopTolerance(Degrees.of(1))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry("LQRArm", TelemetryVerbosity.HIGH);
        
        motor = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), config);
        arm = new Arm(motor);
    }
    
    public Command goToAngle(Angle target) {
        return arm.goTo(() -> target);
    }
    
    @Override
    public void periodic() {
        // LQR runs here via iterateClosedLoop
        motor.iterateClosedLoop();
    }
}
```

{% hint style="danger" %}
**Critical**: You must call `motor.iterateClosedLoop()` in your periodic method for LQR (and any RoboRIO-side control) to execute. This is handled automatically if you use the `Arm`, `Elevator`, or other mechanism classes.
{% endhint %}

## Troubleshooting LQR

### Oscillation

**Symptoms**: Mechanism oscillates around setpoint
**Solutions**:
- Increase R (control cost) to reduce aggressiveness
- Increase velocity cost in Q matrix
- Add or increase feedforward kD term
- Check for mechanical issues (backlash, friction)

### Slow Response

**Symptoms**: Mechanism responds too slowly to setpoint changes
**Solutions**:
- Decrease R (control cost)
- Increase position cost in Q matrix
- Verify motion profile constraints aren't too conservative

### Steady-State Error

**Symptoms**: Mechanism doesn't quite reach setpoint
**Solutions**:
- Verify feedforward is properly tuned (especially kG for arms, kG for elevators)
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
| **Parameters** | kP, kI, kD | Q matrix, R matrix |
| **States Controlled** | Typically one (position or velocity) | Multiple (position AND velocity) |
| **Model Required** | No | Yes (motor, gearing, inertia) |
| **Optimality** | Not guaranteed | Mathematically optimal |
| **Hardware Support** | Motor controllers | RoboRIO only |
| **Update Rate** | 1kHz+ (on controller) | 50Hz (RoboRIO) |
| **Best For** | Simple, fast loops | Complex, model-known systems |
