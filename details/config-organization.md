---
icon: folder-tree
---

# Organizing Configs in Constants

As your robot grows, managing configuration across multiple subsystems can become unwieldy. YAMS supports a clean pattern where you define all your configs in a centralized Constants file, then bind them to subsystems at construction time using `.withSubsystem()`.

## The Problem

Without organization, each subsystem file contains its own configuration inline:

```java
// ArmSubsystem.java - config mixed with subsystem logic
public class ArmSubsystem extends SubsystemBase {
  private final Arm arm;
  
  public ArmSubsystem() {
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
        .withClosedLoopController(5, 0, 0.1)
        .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
        .withTrapezoidalProfile(1.0, 2.0);
    
    SmartMotorController smc = new TalonFXWrapper(new TalonFX(1), DCMotor.getKrakenX60(1), smcConfig);
    
    ArmConfig armConfig = new ArmConfig(smc)
        .withLength(Inches.of(18))
        .withMass(Pounds.of(5))
        // ... more config
    
    this.arm = new Arm(armConfig);
  }
}
```

This approach has drawbacks:
- Configs are scattered across many files
- Hard to compare settings between mechanisms
- Tuning requires opening multiple files
- CAN IDs, motor types, and gear ratios are buried in subsystem code

## The Solution: Centralized Constants with `withSubsystem()`

YAMS configs can be created **without** a subsystem reference, then bound later using `.withSubsystem()`. This enables a clean separation:

```java
// Constants.java - ALL configs in one place
public final class Constants {
  
  public static final class ArmConstants {
    public static final int CAN_ID = 1;
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()  // No subsystem yet!
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
            .withClosedLoopController(5, 0, 0.1)
            .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
            .withTrapezoidalProfile(1.0, 2.0);
    
    public static final ArmConfig ARM_CONFIG = 
        new ArmConfig()  // No SmartMotorController yet!
            .withLength(Inches.of(18))
            .withMass(Pounds.of(5))
            .withHardLimit(Rotations.of(-0.3), Rotations.of(0.3))
            .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
            .withStartingPosition(Rotations.of(0))
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);
  }
  
  public static final class ElevatorConstants {
    public static final int CAN_ID = 2;
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
            .withMechanismCircumference(Inches.of(1.5 * Math.PI))
            .withClosedLoopController(10, 0, 0.5)
            .withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.5, 0.01))
            .withTrapezoidalProfile(1.0, 2.0);
    
    public static final ElevatorConfig ELEVATOR_CONFIG = 
        new ElevatorConfig()
            .withDrumRadius(Inches.of(0.75))
            .withMass(Pounds.of(10))
            .withHardLimits(Meters.of(0), Meters.of(1.5))
            .withSoftLimits(Meters.of(0.02), Meters.of(1.2))
            .withStartingHeight(Meters.of(0.5))
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH);
  }
  
  public static final class ShooterConstants {
    public static final int CAN_ID = 3;
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withClosedLoopController(0.5, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01));
    
    public static final FlyWheelConfig FLYWHEEL_CONFIG = 
        new FlyWheelConfig()
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(0.5))
            .withSoftLimit(RPM.of(0), RPM.of(6000))
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
  }
}
```

Then in your subsystems, use `.withSubsystem()` to bind the config:

```java
// ArmSubsystem.java - clean and focused
public class ArmSubsystem extends SubsystemBase {
  private final Arm arm;
  
  public ArmSubsystem() {
    // Bind the config to this subsystem
    SmartMotorControllerConfig smcConfig = ArmConstants.SMC_CONFIG
        .withSubsystem(this);
    
    // Create SmartMotorController with the bound config
    SmartMotorController smc = new TalonFXWrapper(
        new TalonFX(ArmConstants.CAN_ID), 
        DCMotor.getKrakenX60(1), 
        smcConfig
    );
    
    // Bind ArmConfig to the SmartMotorController
    ArmConfig armConfig = ArmConstants.ARM_CONFIG
        .withSmartMotorController(smc);
    
    this.arm = new Arm(armConfig);
  }
  
  // Commands and business logic only - no config clutter!
  public Command goToAngle(Angle target) {
    return arm.runTo(target);
  }
}
```

## Benefits of This Pattern

| Benefit | Description |
|---------|-------------|
| **Single Source of Truth** | All CAN IDs, gear ratios, PID gains in one file |
| **Easy Comparison** | See all mechanism configs side-by-side |
| **Faster Tuning** | Change gains without navigating subsystem files |
| **Cleaner Subsystems** | Subsystem classes focus on behavior, not configuration |
| **Reusable Configs** | Share base configs between similar mechanisms |

## Complete Example: Constants File

```java
package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public final class Constants {

  // ==================== ARM ====================
  public static final class ArmConstants {
    public static final int CAN_ID = 1;
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
            .withClosedLoopController(5, 0, 0.1)
            .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
            .withTrapezoidalProfile(1.0, 2.0)
            .withIdleMode(MotorMode.BRAKE);
    
    public static final ArmConfig ARM_CONFIG = 
        new ArmConfig()
            .withLength(Inches.of(18))
            .withMass(Pounds.of(5))
            .withHardLimit(Rotations.of(-0.3), Rotations.of(0.3))
            .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
            .withStartingPosition(Rotations.of(0))
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);
  }

  // ==================== ELEVATOR ====================
  public static final class ElevatorConstants {
    public static final int CAN_ID = 2;
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
            .withMechanismCircumference(Inches.of(1.5 * Math.PI))
            .withClosedLoopController(10, 0, 0.5)
            .withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.5, 0.01))
            .withTrapezoidalProfile(1.0, 2.0)
            .withIdleMode(MotorMode.BRAKE);
    
    public static final ElevatorConfig ELEVATOR_CONFIG = 
        new ElevatorConfig()
            .withDrumRadius(Inches.of(0.75))
            .withMass(Pounds.of(10))
            .withHardLimits(Meters.of(0), Meters.of(1.5))
            .withSoftLimits(Meters.of(0.02), Meters.of(1.2))
            .withStartingHeight(Meters.of(0.5))
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH);
  }

  // ==================== SHOOTER ====================
  public static final class ShooterConstants {
    public static final int UPPER_CAN_ID = 3;
    public static final int LOWER_CAN_ID = 4;
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withClosedLoopController(0.5, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01))
            .withIdleMode(MotorMode.COAST);
    
    public static final FlyWheelConfig FLYWHEEL_CONFIG = 
        new FlyWheelConfig()
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(0.5))
            .withSoftLimit(RPM.of(0), RPM.of(6000))
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
  }
}
```

## Complete Example: Subsystem Using Constants

```java
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ArmSubsystem extends SubsystemBase {

  private final Arm arm;

  public ArmSubsystem() {
    // Step 1: Bind SmartMotorControllerConfig to this subsystem
    SmartMotorControllerConfig smcConfig = ArmConstants.SMC_CONFIG
        .withSubsystem(this);

    // Step 2: Create SmartMotorController
    SmartMotorController smc = new TalonFXWrapper(
        new TalonFX(ArmConstants.CAN_ID),
        ArmConstants.MOTOR,
        smcConfig
    );

    // Step 3: Bind ArmConfig to SmartMotorController
    ArmConfig armConfig = ArmConstants.ARM_CONFIG
        .withSmartMotorController(smc);

    // Step 4: Create Arm mechanism
    this.arm = new Arm(armConfig);
  }

  // ==================== COMMANDS ====================
  
  public Command runToAngle(Angle target) {
    return arm.runTo(target);
  }

  public Command holdAngle(Angle target) {
    return arm.run(target);
  }

  public Command stow() {
    return arm.runTo(Rotations.of(0));
  }

  // ==================== ACCESSORS ====================
  
  public Arm getArm() {
    return arm;
  }
}
```

## Organizing Multiple Config Files

For larger robots, you may want to split configs into separate files:

```
frc/robot/
├── Constants.java           // General constants (CAN IDs, ports)
├── configs/
│   ├── ArmConfigs.java      // Arm-related configs
│   ├── ElevatorConfigs.java // Elevator-related configs
│   ├── ShooterConfigs.java  // Shooter-related configs
│   └── DriveConfigs.java    // Swerve/drive configs
└── subsystems/
    ├── ArmSubsystem.java
    ├── ElevatorSubsystem.java
    └── ...
```

{% hint style="info" %}
**Important**: When using this pattern, you **must** call `.withSubsystem()` before passing the config to a `SmartMotorController`, and `.withSmartMotorController()` before passing to a Mechanism class. YAMS will throw an exception if you forget.
{% endhint %}

## When NOT to Use This Pattern

This pattern adds a layer of indirection. For simple robots with 1-2 mechanisms, inline configuration may be clearer. Consider using centralized configs when:

- You have 3+ mechanisms
- Multiple team members tune different subsystems
- You want to quickly compare settings across mechanisms
- You're building a constants dashboard or config loader
