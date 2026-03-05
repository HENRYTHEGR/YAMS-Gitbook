---
description: >-
  System Identification (SysId) is the process of determining feedforward constants
  for your mechanism through statistical analysis of its inputs and outputs.
---

# System Identification (SysId)

## What is System Identification?

System Identification is the process of determining a mathematical model for the behavior of a system through statistical analysis of its inputs and outputs. This model describes how input voltage affects the way your measurements (typically encoder data) evolve over time.

The WPILib SysId tool takes a model and dataset and attempts to fit parameters which would make your model most closely match the dataset. Even an imperfect model is usually "good enough" to give accurate **feedforward control** of the mechanism, and even to estimate optimal gains for **feedback control**.

{% hint style="info" %}
For a complete understanding of SysId theory, visit the [WPILib System Identification Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html).
{% endhint %}

## The Feedforward Equations

Different mechanisms use different feedforward equations:

### Simple Motor (Flywheels, Turrets, Linear Sliders)

$$V = K_s \cdot sgn(\dot{d}) + K_v \cdot \dot{d} + K_a \cdot \ddot{d}$$

### Elevator

$$V = K_g + K_s \cdot sgn(\dot{d}) + K_v \cdot \dot{d} + K_a \cdot \ddot{d}$$

The constant term ($$K_g$$) accounts for gravity.

### Arm

$$V = K_g \cdot cos(\theta) + K_s \cdot sgn(\dot{\theta}) + K_v \cdot \dot{\theta} + K_a \cdot \ddot{\theta}$$

The cosine term ($$K_g$$) accounts for the effect of gravity at different arm angles.

## Types of Tests

A standard SysId routine consists of **two types of tests**, each run in **both directions** (forward and backward):

| Test Type | Description |
|-----------|-------------|
| **Quasistatic** | The mechanism is gradually sped up such that the voltage corresponding to acceleration is negligible (hence, "as if static"). This helps determine $$K_s$$ and $$K_v$$. |
| **Dynamic** | A constant "step voltage" is applied to the mechanism, so that the behavior while accelerating can be determined. This helps determine $$K_a$$. |

{% hint style="warning" %}
Running a "backwards" test directly after a "forwards" test is generally advisable as it will more or less reset the mechanism to its original position.
{% endhint %}

---

## Using YAMS Helper Functions

YAMS provides built-in `sysId()` methods on mechanism classes (`Arm`, `Elevator`, `Shooter`, etc.) that simplify the entire process.

{% hint style="danger" %}
This section covers SysId with REV Spark or ThriftyNova Motor Controllers.
For CTRE TalonFX and TalonFXS, see the [CTRE SysId with Signal Logger](#ctre-sysid-with-signal-logger) section below.
{% endhint %}

### Prerequisites

Before running SysId:

* Soft and hard limits are already configured
* Gearing is correct
* Mechanism circumference is set (for linear mechanisms)

### Arm Example

```java
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class ArmSubsystem extends SubsystemBase {

  private Arm arm; // Your configured YAMS Arm

  /**
   * Run SysId on the Arm.
   * 
   * Static test: runs the arm up and down with 7V step voltage.
   * Dynamic test: ramps from 0V to 7V at 2V per second.
   * Test duration: 4 seconds maximum.
   */
  public Command sysId() { 
    return arm.sysId(
      Volts.of(7),              // Step voltage for dynamic test
      Volts.of(2).per(Second),  // Ramp rate for quasistatic test
      Seconds.of(4)             // Maximum test duration
    );
  }
  
  // ... rest of subsystem
}
```

### Elevator Example

```java
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class ElevatorSubsystem extends SubsystemBase {

  private Elevator elevator; // Your configured YAMS Elevator

  /**
   * Run SysId on the Elevator.
   */
  public Command sysId() { 
    return elevator.sysId(
      Volts.of(7),              // Step voltage
      Volts.of(2).per(Second),  // Ramp rate
      Seconds.of(4)             // Timeout
    );
  }
  
  // ... rest of subsystem
}
```

### Binding to Controller Buttons

```java
public class RobotContainer {

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final CommandXboxController controller = 
      new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
    
    // Default command to hold position
    armSubsystem.setDefaultCommand(armSubsystem.set(0));
  }

  private void configureBindings() {
    // Run SysId while A is held - release to stop
    controller.a().whileTrue(armSubsystem.sysId());
    
    // Manual control for testing
    controller.x().whileTrue(armSubsystem.set(0.3));
    controller.y().whileTrue(armSubsystem.set(-0.3));
  }
}
```

{% hint style="info" %}
Use `whileTrue()` so you can release the button to immediately stop the test if the mechanism exceeds safe limits!
{% endhint %}

---

## Using WPILib SysIdRoutine Directly

If you need more control over the SysId process, or are working with a `SmartMotorController` directly without a mechanism class, you can use WPILib's `SysIdRoutine` directly.

{% embed url="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html" %}

### Creating the SysIdRoutine

The `SysIdRoutine` requires two objects:

1. **Config** - Test settings (ramp rate, step voltage, timeout)
2. **Mechanism** - Callbacks for driving motors and logging data

{% hint style="info" %}
**Why use duty cycle instead of voltage control?**

Modern motor controllers (SparkMax, SparkFlex, TalonFX, etc.) have internal closed-loop voltage controllers. When you call a "set voltage" API, the motor controller actively adjusts output to maintain that voltage regardless of load or battery sag.

For SysId, we want **raw, uncompensated motor behavior**. By converting voltage to duty cycle (`voltage / batteryVoltage`) and using `setDutyCycle()`, we bypass the internal controller entirely. This produces cleaner data that more accurately represents the true motor and mechanism dynamics.
{% endhint %}

{% hint style="info" %}
**Why use `MutVoltage`, `MutAngle`, and `MutAngularVelocity`?**

The logging callback runs every robot loop iteration (typically 50Hz). Creating new `Voltage`, `Angle`, or `AngularVelocity` objects each iteration would allocate memory on the heap, increasing garbage collection (GC) pressure and potentially causing loop overruns.

The `Mut*` (mutable) variants allow you to reuse the same object and update its value in-place with `mut_replace()`. This eliminates per-iteration allocations and keeps your robot code running smoothly.
{% endhint %}

{% hint style="warning" %}
**Important: `updateTelemetry()` and `simIterate()` in the log callback**

The log callback calls `motor.updateTelemetry()` and `motor.simIterate()` to ensure sensor data (position, velocity) is accurate at the exact moment of logging. This is critical for simulation accuracy.

**When running SysId manually**, you should **temporarily disable or remove** any `updateTelemetry()` and `simIterate()` calls from your subsystem's `periodic()` method. Having these calls in both places can cause duplicate updates per loop iteration, which may produce inconsistent data.
{% endhint %}

```java
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import yams.motorcontrollers.SmartMotorController;

public class ManualSysIdSubsystem extends SubsystemBase {

  private SmartMotorController motor; // Your YAMS SmartMotorController

  // Mutable holders for unit types - reused each loop iteration to avoid
  // allocating new objects on the heap during logging (reduces GC pressure)
  private final MutVoltage m_appliedVoltage = new MutVoltage(0, 0, Volts);
  private final MutAngle m_position = new MutAngle(0, 0, Rotations);
  private final MutAngularVelocity m_velocity = new MutAngularVelocity(0, 0, RotationsPerSecond);

  // Create the SysIdRoutine
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      // Config: ramp rate, step voltage, timeout
      new SysIdRoutine.Config(
          Volts.of(1).per(Seconds.of(1)),  // Quasistatic ramp rate (1 V/s)
          Volts.of(4),                      // Dynamic step voltage
          Seconds.of(10)                    // Timeout
      ),
      new SysIdRoutine.Mechanism(
          // Drive callback - convert voltage to duty cycle
          // Using duty cycle instead of the motor controller's voltage control
          // bypasses the internal closed-loop controller, resulting in cleaner data
          (Voltage voltage) -> motor.setDutyCycle(
              voltage.in(Volts) / RobotController.getBatteryVoltage()
          ),
          // Log callback - records position, velocity, and voltage
          // updateTelemetry() and simIterate() ensure sensor data is fresh at logging time
          log -> {
            motor.updateTelemetry();
            motor.simIterate();
            log.motor("motor")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        motor.getDutyCycle() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_position.mut_replace(motor.getMechanismPosition()))
                .angularVelocity(m_velocity.mut_replace(motor.getMechanismVelocity()));
          },
          this,           // Subsystem for requirements
          "MyMechanism"   // Name for logging
      )
  );

  /**
   * Returns the quasistatic test command.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns the dynamic test command.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
```

### Reusable SysIdRoutine Factory Method

You can create a static helper method to generate `SysIdRoutine` instances for any `SmartMotorController`:

```java
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import yams.motorcontrollers.SmartMotorController;

public class SystemIdentification {

  private static final MutVoltage m_appliedVoltage = new MutVoltage(0, 0, Volts);
  private static final MutAngle m_distance = new MutAngle(0, 0, Rotations);
  private static final MutAngularVelocity m_velocity = 
      new MutAngularVelocity(0, 0, RotationsPerSecond);

  /**
   * Creates a SysIdRoutine for a SmartMotorController.
   *
   * @param motor         The SmartMotorController to characterize
   * @param motorName     Name for logging (appears in SysId tool)
   * @param subsystemBase The subsystem that owns this motor
   * @return A configured SysIdRoutine
   */
  public static SysIdRoutine createRoutine(
      SmartMotorController motor, 
      String motorName, 
      SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Seconds.of(1)),  // Ramp rate: 1 V/s
            Volts.of(4),                      // Step voltage: 4 V
            Seconds.of(10)                    // Timeout: 10 s
        ),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> motor.setDutyCycle(
                voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              motor.updateTelemetry();
              motor.simIterate();
              log.motor(motorName)
                  .voltage(m_appliedVoltage.mut_replace(
                      motor.getDutyCycle() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_distance.mut_replace(motor.getMechanismPosition()))
                  .angularVelocity(m_velocity.mut_replace(motor.getMechanismVelocity()));
            },
            subsystemBase,
            motorName));
  }
}
```

Then use it in your subsystem:

```java
public class ShooterSubsystem extends SubsystemBase {
  
  private SmartMotorController shooterMotor;
  private final SysIdRoutine sysIdRoutine;
  
  public ShooterSubsystem() {
    // ... motor configuration ...
    sysIdRoutine = SystemIdentification.createRoutine(shooterMotor, "Shooter", this);
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
```

### Binding All Four Tests

When using `SysIdRoutine` directly, you need to bind all four tests separately:

```java
private void configureBindings() {
  // Quasistatic tests
  controller.a().whileTrue(subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
  controller.b().whileTrue(subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  
  // Dynamic tests  
  controller.x().whileTrue(subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
  controller.y().whileTrue(subsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
}
```

{% hint style="warning" %}
Only log files with a **single routine** in them are usable for analysis. If you run a routine on one motor and then run a routine on another motor without extracting the log or power-cycling the roboRIO in between, analysis will fail.
{% endhint %}

### Running All Tests with One Command

You can combine all four tests into a single command sequence that runs automatically:

```java
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public static Command getFullSysIdCommand(
    SysIdRoutine routine,
    double quasistaticTimeout,
    double dynamicTimeout,
    double delayBetweenTests) {
  
  return routine.quasistatic(SysIdRoutine.Direction.kForward)
      .withTimeout(quasistaticTimeout)
      .andThen(Commands.waitSeconds(delayBetweenTests))
      .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse)
          .withTimeout(quasistaticTimeout))
      .andThen(Commands.waitSeconds(delayBetweenTests))
      .andThen(routine.dynamic(SysIdRoutine.Direction.kForward)
          .withTimeout(dynamicTimeout))
      .andThen(Commands.waitSeconds(delayBetweenTests))
      .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse)
          .withTimeout(dynamicTimeout));
}

// Usage:
controller.a().whileTrue(
    getFullSysIdCommand(sysIdRoutine, 4.0, 2.0, 1.0)
);
```

{% hint style="warning" %}
**Why the delay between tests matters**

When a test completes, the mechanism still has momentum (kinetic energy) from the previous motion. If you immediately start a test in the opposite direction, the motor must first counteract this residual momentum before the new test's motion begins. This "fighting against itself" period produces misleading data that can skew your feedforward constants.

For example, if a flywheel is spinning at high speed after a dynamic forward test and you immediately run dynamic reverse, the initial voltage readings will include the energy spent decelerating the flywheel - not just accelerating it in the new direction.

**Recommendations:**
- Use at least **1-2 seconds** of delay between tests for most mechanisms
- For high-inertia mechanisms (heavy arms, large flywheels), use **3+ seconds** or wait until the mechanism has fully stopped
- Consider adding a `stopMotor` call between tests to actively brake the mechanism
- If running tests manually with separate button bindings, wait for the mechanism to settle before starting the next test
{% endhint %}

For mechanisms with limits (arms, elevators), you can add safety stops:

```java
public static Command getFullSysIdCommand(
    SysIdRoutine routine,
    double quasistaticTimeout,
    double dynamicTimeout,
    double delay,
    BooleanSupplier isAtMax,
    BooleanSupplier isAtMin,
    Runnable stopMotor) {
  
  return routine.quasistatic(SysIdRoutine.Direction.kForward)
      .until(isAtMax)
      .finallyDo(stopMotor)
      .withTimeout(quasistaticTimeout)
      .andThen(Commands.waitSeconds(delay))
      .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse)
          .until(isAtMin)
          .finallyDo(stopMotor)
          .withTimeout(quasistaticTimeout))
      .andThen(Commands.waitSeconds(delay))
      .andThen(routine.dynamic(SysIdRoutine.Direction.kForward)
          .until(isAtMax)
          .finallyDo(stopMotor)
          .withTimeout(dynamicTimeout))
      .andThen(Commands.waitSeconds(delay))
      .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse)
          .until(isAtMin)
          .finallyDo(stopMotor)
          .withTimeout(dynamicTimeout));
}
```

---

## Running the Tests

1. **Deploy your code** to the robot
2. **Ensure sufficient space** - at least 10-20 feet for drivetrains, adequate range of motion for arms/elevators
3. **Run all four tests** in sequence:
   - Quasistatic Forward
   - Quasistatic Reverse
   - Dynamic Forward
   - Dynamic Reverse
4. **Monitor carefully** - stop tests early if the mechanism exceeds safe limits

{% hint style="danger" %}
Watch out for your mechanism and stop the test early if it exceeds safe limits! The routine only creates voltage commands - it is up to you to set up hard or soft limits to prevent injury or damage.
{% endhint %}

---

## Analyzing Results

After running all tests:

1. **Retrieve the log file** from the roboRIO using the DataLogTool
2. **Open SysId** (included with WPILib)
3. **Load the log file** in the Log Loader pane
4. **Drag entries** to the Data Selector:
   - Find an entry containing "state" (type: string) → Test State slot
   - Find Position, Velocity, and Voltage entries → respective slots
5. **Select analysis type** (Simple, Elevator, or Arm)
6. **Click Load** and review diagnostics

{% embed url="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/loading-data.html" %}

### Applying the Results

Once you have your feedforward constants ($$K_s$$, $$K_v$$, $$K_a$$, and $$K_g$$ if applicable), apply them to your YAMS configuration:

```java
// For Arms
.withFeedforward(new ArmFeedforward(kS, kG, kV, kA))

// For Elevators  
.withFeedforward(new ElevatorFeedforward(kS, kG, kV, kA))

// For Simple Motors (Shooters, Turrets)
.withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
```

{% hint style="success" %}
Remember to also set `.withSimFeedforward()` with the same or similar values for accurate simulation!
{% endhint %}

---

## YAMS vs Manual: When to Use Each

| Approach | Best For |
|----------|----------|
| **YAMS `mechanism.sysId()`** | Standard mechanisms (Arm, Elevator, Shooter) where you're already using YAMS mechanism classes. Simplest setup. |
| **WPILib `SysIdRoutine`** | Loosely coupled mechanisms, custom configurations, or when you need fine-grained control over the logging process. |

Both approaches produce compatible log files that work with the WPILib SysId analysis tool.

---

## REV SysId with Status Logger

When using REV motor controllers (SPARK MAX, SPARK Flex), YAMS leverages the **REV Status Logger** to capture high-quality CAN status frame data. This runs automatically in the background and can provide better data than the standard WPILib DataLog approach.

{% embed url="https://docs.revrobotics.com/revlib/logs" %}

### How Status Logger Works

Status Logger is **enabled by default** in REVLib 2026+ and requires no setup. When you run SysId tests with REV motor controllers:

1. The standard WPILib `.wpilog` file is written to the roboRIO (usable directly in SysId)
2. A separate `.revlog` file is also written, containing raw CAN status frames with higher fidelity

{% hint style="info" %}
**Do I need the `.revlog` file?**

The standard `.wpilog` file from your SysId routine **may work fine** for analysis. However, the `.revlog` file captures raw CAN data at a higher rate and avoids timing issues from the robot loop. For the best results, especially if you encounter issues with your initial analysis, extract and convert the `.revlog` file.
{% endhint %}

### Log File Locations

Status Logger saves `.revlog` files to:

| Storage | Path |
|---------|------|
| **roboRIO internal** | `/home/lvuser/logs/` |
| **USB drive** (recommended) | `/u/logs/` |

{% hint style="warning" %}
It is recommended to insert an external USB drive for storing `.revlog` files. This reduces clutter on the roboRIO and makes file retrieval easier.
{% endhint %}

### Retrieving `.revlog` Files

**From roboRIO internal storage:**

Use FTP to access files at `/home/lvuser/logs/`. See the [WPILib FTP documentation](https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ftp.html) for details.

**From USB drive:**

Either access the `/u/logs/` directory with the flash drive plugged into the roboRIO, or simply unplug the USB drive and insert it into your computer.

### Converting `.revlog` to `.wpilog`

`.revlog` files must be converted before use in SysId. There are two methods:

#### Method 1: AdvantageScope (Recommended)

1. Open **AdvantageScope**
2. Open your `.revlog` file directly
3. AdvantageScope automatically converts it to `.wpilog` format
4. Export as `.wpilog` if needed for SysId

{% hint style="success" %}
AdvantageScope is the easiest method and handles the conversion automatically when you open the file.
{% endhint %}

#### Method 2: revlog-converter CLI

Use the `revlog-converter` NPM package for command-line conversion:

```bash
# Install globally
npm install -g revlog-converter

# Convert a file
revlog-converter mylog.revlog --output mylog.wpilog
```

### Manual Logging Control

If you need precise control over when logging occurs (e.g., only during SysId tests), you can disable auto-logging:

```java
import com.revrobotics.util.StatusLogger;

@Override
public void robotInit() {
  // Disable automatic logging - must be called before any REVLib device is created
  StatusLogger.disableAutoLogging();
}

// Start logging when running SysId
public Command sysIdWithLogging() {
  return Commands.runOnce(StatusLogger::start)
      .andThen(mechanism.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4)))
      .finallyDo(() -> StatusLogger.stop());
}
```

{% hint style="warning" %}
`disableAutoLogging()` must be called **before** any REVLib device object is created (e.g., before constructing any `SparkMax` or `SparkFlex`). The recommended placement is as the first line in `robotInit()`.
{% endhint %}

---

## CTRE SysId with Signal Logger

When using CTRE motor controllers (TalonFX, TalonFXS), YAMS leverages the **Phoenix 6 Signal Logger** instead of WPILib's DataLog. This provides several advantages:

- Eliminates CAN latency issues
- Supports signals faster than the 20ms main robot loop
- Avoids Java garbage collection pauses affecting log quality

{% embed url="https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html" %}

### How YAMS Uses Signal Logger

When you call `mechanism.sysId()` with a CTRE motor controller, YAMS automatically:

1. Configures the `SysIdRoutine` to log state using `SignalLogger.writeString()`
2. Sets the log callback to `null` (Signal Logger captures all signals automatically)
3. Uses `VoltageOut` control requests to apply voltage during tests

The resulting routine looks like this internally:

```java
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

private final TalonFX motor = new TalonFX(0);
private final VoltageOut voltageRequest = new VoltageOut(0.0);

private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
        null,           // Use default ramp rate (1 V/s)
        Volts.of(4),    // Reduce dynamic step voltage to prevent brownout
        null,           // Use default timeout (10 s)
        // Log state with Phoenix SignalLogger
        (state) -> SignalLogger.writeString("state", state.toString())
    ),
    new SysIdRoutine.Mechanism(
        (volts) -> motor.setControl(voltageRequest.withOutput(volts.in(Volts))),
        null,  // No log callback - Signal Logger handles this automatically
        this
    )
);
```

### Running CTRE SysId Tests

You must **manually start and stop the Signal Logger** before and after running tests:

```java
// In RobotContainer.java
controller.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
controller.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

// Bind the four SysId tests
controller.y().whileTrue(mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
controller.a().whileTrue(mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
controller.b().whileTrue(mechanism.sysIdDynamic(SysIdRoutine.Direction.kForward));
controller.x().whileTrue(mechanism.sysIdDynamic(SysIdRoutine.Direction.kReverse));
```

{% hint style="warning" %}
**Important:** Start the Signal Logger **before** running any tests and stop it **after** all four tests are complete. This ensures all test data is captured in a single log file without extraneous data from other robot actions.
{% endhint %}

### Extracting and Converting CTRE Logs

CTRE logs are saved in the `.hoot` format, which must be converted to `.wpilog` before analysis in SysId.

{% hint style="danger" %}
**Do not use third-party tools** to convert `.hoot` files. A lossy conversion can impact the quality of SysId analysis, especially in simulation where it may cause SysId to fail entirely.
{% endhint %}

#### Step 1: Locate the Log File

After running your tests, the `.hoot` log file is stored on the roboRIO at:

```
/home/lvuser/logs/
```

You can also find logs on a USB drive if one was connected during testing.

#### Step 2: Extract Using Phoenix Tuner X

1. Open **Phoenix Tuner X**
2. Connect to your robot
3. Navigate to **Tools → Extracting Signal Logs**
4. Select your `.hoot` log file
5. Choose **WPILOG** as the export format
6. Save the exported `.wpilog` file

{% embed url="https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tools/log-extractor.html" %}

#### Step 3: Alternative - Using owlet CLI

You can also use CTRE's `owlet` command-line tool:

```bash
# Install owlet (if not already installed)
# Available from CTRE's GitHub releases

# Convert hoot to wpilog
owlet convert mylog.hoot --format wpilog --output mylog.wpilog
```

#### Step 4: Load in SysId

1. Open the **SysId** tool (included with WPILib)
2. In the **Log Loader** pane, load your converted `.wpilog` file
3. In the **Data Selector**, drag the following signals:
   - `state` (string) → **Test State** slot
   - TalonFX `Position` → **Position** slot
   - TalonFX `Velocity` → **Velocity** slot
   - TalonFX `MotorVoltage` → **Voltage** slot
4. Select your analysis type (Simple, Elevator, or Arm)
5. Click **Load** and review the diagnostics

{% hint style="info" %}
The Signal Logger automatically captures `Position`, `Velocity`, and `MotorVoltage` signals from your TalonFX motors - you don't need to manually log these values like you would with REV motors.
{% endhint %}

### CTRE vs REV: Key Differences

| Aspect | CTRE (TalonFX) | REV (SparkMax/Flex) |
|--------|----------------|---------------------|
| **Primary Logging** | Phoenix Signal Logger (`.hoot`) | WPILib DataLog (`.wpilog`) + Status Logger (`.revlog`) |
| **Log Callback** | `null` (automatic) | Manual logging required |
| **State Logging** | `SignalLogger.writeString()` | Default WPILib logger |
| **Conversion Required** | Yes - Tuner X or owlet | Optional - `.wpilog` works, `.revlog` is better |
| **Signal Quality** | Higher (bypasses CAN latency) | Standard loop or higher with `.revlog` |
| **Setup Required** | Must start/stop SignalLogger | Status Logger runs automatically |

---

## Related Documentation

* [How to run SysId on an Arm](../how-to/how-to-run-sysid-on-a-arm.md)
* [How to run SysId on an Elevator](../how-to/how-to-run-sysid-on-a-elevator.md)
* [How do I control a Mechanism without a Mechanism Class?](../how-to/how-do-i-control-a-mechanism-without-a-mechanism-class.md)
* [WPILib SysId Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html)
* [REV Status Logger Documentation](https://docs.revrobotics.com/revlib/logs)
* [CTRE Phoenix 6 SysId Integration](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/index.html)
* [CTRE Extracting Signal Logs](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tools/log-extractor.html)
