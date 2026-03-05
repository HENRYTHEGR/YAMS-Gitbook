# Swerve Drive

This tutorial covers how to create a swerve drive using YAMS. YAMS provides a complete swerve drive implementation with built-in simulation, odometry, and telemetry.

## Overview

YAMS swerve consists of three main components:
- `SwerveModule` - Individual module with drive and azimuth motors
- `SwerveDrive` - The complete drive system managing all modules
- `SwerveInputStream` - Helper for converting controller inputs to chassis speeds

## Creating a Swerve Module

Each swerve module requires two `SmartMotorController`s (drive and azimuth) and a `SwerveModuleConfig`:

```java
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public SwerveModule createModule(
    SparkMax driveMotor, 
    SparkMax azimuthMotor, 
    CANcoder absoluteEncoder, 
    String moduleName,
    Translation2d location) 
{
  // Define gearing ratios
  MechanismGearing driveGearing = new MechanismGearing(12.75);  // L2 SDS MK4i
  MechanismGearing azimuthGearing = new MechanismGearing(6.75);
  Distance wheelDiameter = Inches.of(4);

  // Drive motor configuration
  SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
      .withWheelDiameter(wheelDiameter)
      .withClosedLoopController(0.3, 0, 0)
      .withGearing(driveGearing)
      .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / 4.5, 0.01))  // kS, kV, kA
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);

  // Azimuth motor configuration
  SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
      .withClosedLoopController(1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0, 1))
      .withGearing(azimuthGearing)
      .withStatorCurrentLimit(Amps.of(20))
      .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);

  // Create SmartMotorControllers
  SmartMotorController driveSMC = new SparkWrapper(driveMotor, DCMotor.getNEO(1), driveCfg);
  SmartMotorController azimuthSMC = new SparkWrapper(azimuthMotor, DCMotor.getNEO(1), azimuthCfg);

  // Create module configuration
  SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
      .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
      .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
      .withLocation(location)
      .withOptimization(true);  // Enable state optimization

  return new SwerveModule(moduleConfig);
}
```

### Module Configuration Options

| Method | Description |
|--------|-------------|
| `withAbsoluteEncoder(Supplier<Angle>)` | Absolute encoder for azimuth position |
| `withLocation(Translation2d)` | Module position relative to robot center |
| `withOptimization(boolean)` | Enable state optimization (recommended) |
| `withTelemetry(String, TelemetryVerbosity)` | Configure telemetry output |

## Creating the SwerveDrive

Once you have your modules, create the `SwerveDrive`:

```java
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive drive;

  public SwerveSubsystem() {
    Pigeon2 gyro = new Pigeon2(14);

    // Create modules (FL, FR, BL, BR order)
    var fl = createModule(
        new SparkMax(1, MotorType.kBrushless),
        new SparkMax(2, MotorType.kBrushless),
        new CANcoder(3),
        "frontleft",
        new Translation2d(Inches.of(12), Inches.of(12)));
    
    var fr = createModule(
        new SparkMax(4, MotorType.kBrushless),
        new SparkMax(5, MotorType.kBrushless),
        new CANcoder(6),
        "frontright",
        new Translation2d(Inches.of(12), Inches.of(-12)));
    
    var bl = createModule(
        new SparkMax(7, MotorType.kBrushless),
        new SparkMax(8, MotorType.kBrushless),
        new CANcoder(9),
        "backleft",
        new Translation2d(Inches.of(-12), Inches.of(12)));
    
    var br = createModule(
        new SparkMax(10, MotorType.kBrushless),
        new SparkMax(11, MotorType.kBrushless),
        new CANcoder(12),
        "backright",
        new Translation2d(Inches.of(-12), Inches.of(-12)));

    // Create SwerveDriveConfig
    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro.getYaw().asSupplier())
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));

    drive = new SwerveDrive(config);
  }
}
```

### SwerveDriveConfig Options

| Method | Description |
|--------|-------------|
| `withGyro(Supplier<Angle>)` | Gyroscope angle supplier |
| `withStartingPose(Pose2d)` | Initial robot pose |
| `withTranslationController(PIDController)` | PID for drive-to-pose translation |
| `withRotationController(PIDController)` | PID for drive-to-pose rotation |
| `withGyroOffset(Angle)` | Offset to apply to gyro reading |

## Driving the Robot

### Using SwerveInputStream

`SwerveInputStream` is a powerful helper for converting joystick inputs to `ChassisSpeeds`:

```java
private AngularVelocity maximumAngularVelocity = DegreesPerSecond.of(360);
private LinearVelocity maximumLinearVelocity = MetersPerSecond.of(4);

public SwerveInputStream getChassisSpeedsSupplier(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier rotation) 
{
  return new SwerveInputStream(drive, translationX, translationY, rotation)
      .withMaximumAngularVelocity(maximumAngularVelocity)
      .withMaximumLinearVelocity(maximumLinearVelocity)
      .withDeadband(0.01)
      .withCubeRotationControllerAxis()     // Non-linear rotation response
      .withCubeTranslationControllerAxis()  // Non-linear translation response
      .withAllianceRelativeControl();       // Alliance-aware field-relative
}
```

### SwerveInputStream Options

| Method | Description |
|--------|-------------|
| `withMaximumLinearVelocity(LinearVelocity)` | Max translation speed |
| `withMaximumAngularVelocity(AngularVelocity)` | Max rotation speed |
| `withDeadband(double)` | Controller deadband |
| `withCubeTranslationControllerAxis()` | Cube translation for smoother control |
| `withCubeRotationControllerAxis()` | Cube rotation for smoother control |
| `withAllianceRelativeControl()` | Alliance-aware field-relative driving |
| `withRobotRelative()` | Robot-relative driving |
| `withControllerHeadingAxis(DoubleSupplier, DoubleSupplier)` | Heading-based control |

### Drive Commands

```java
/**
 * Drive with field-relative chassis speeds.
 */
public Command drive(Supplier<ChassisSpeeds> speedsSupplier) {
  return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get()))
      .withName("Field Oriented Drive");
}

/**
 * Drive with robot-relative chassis speeds.
 */
public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier) {
  return drive.drive(speedsSupplier);
}

/**
 * Drive to a specific pose.
 */
public Command driveToPose(Pose2d pose) {
  return drive.driveToPose(pose);
}

/**
 * Lock the wheels in an X pattern to resist pushing.
 */
public Command lock() {
  return run(drive::lockPose);
}
```

## SwerveDrive Methods Reference

### Driving

| Method | Description |
|--------|-------------|
| `drive(Supplier<ChassisSpeeds>)` | Command to drive with robot-relative speeds |
| `setRobotRelativeChassisSpeeds(ChassisSpeeds)` | Set robot-relative speeds directly |
| `setFieldRelativeChassisSpeeds(ChassisSpeeds)` | Set field-relative speeds |
| `setSwerveModuleStates(SwerveModuleState[])` | Set module states directly |
| `lockPose()` | Lock wheels in X pattern |
| `driveToPose(Pose2d)` | Command to drive to a pose |

### Odometry & Pose

| Method | Description |
|--------|-------------|
| `getPose()` | Get current estimated pose |
| `resetOdometry(Pose2d)` | Reset pose estimator |
| `getGyroAngle()` | Get current gyro angle |
| `zeroGyro()` | Zero the gyroscope |
| `addVisionMeasurement(Pose2d, double)` | Add vision measurement |
| `addVisionMeasurement(Pose2d, double, Matrix)` | Add vision measurement with std devs |

### Utility

| Method | Description |
|--------|-------------|
| `getKinematics()` | Get SwerveDriveKinematics |
| `getModuleStates()` | Get current module states |
| `getModulePositions()` | Get current module positions |
| `getRobotRelativeSpeed()` | Get current robot-relative speeds |
| `getFieldRelativeSpeed()` | Get current field-relative speeds |
| `getDistanceFromPose(Pose2d)` | Distance to a pose |
| `getAngleDifferenceFromPose(Pose2d)` | Angle difference to a pose |

## Periodic Updates

You must call these methods in your periodic functions:

```java
@Override
public void periodic() {
  drive.updateTelemetry();  // Updates pose estimator and publishes telemetry
}

@Override
public void simulationPeriodic() {
  drive.simIterate();  // Updates simulation
}
```

## SysId for Characterization

YAMS provides built-in SysId routines for swerve:

```java
/**
 * Run SysId on the azimuth motors.
 */
public Command azimuthSysId() {
  return drive.sysIdAzimuth(
      drive.getModule("frontleft").orElseThrow(),
      Volts.of(3),           // Quasistatic voltage
      Volts.of(1).per(Second), // Dynamic ramp rate
      Second.of(10));         // Timeout
}

/**
 * Run SysId on the drive motors.
 */
public Command driveSysId() {
  return drive.sysIdDrive(
      Volts.of(12),            // Max voltage
      Volts.of(3).per(Second), // Ramp rate
      Second.of(15),           // Timeout
      DriveSysIdTestType.DRIVE);
}
```

## Complete Example

Here's a complete swerve subsystem:

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive drive;
  private final Field2d field = new Field2d();

  private final AngularVelocity maxAngularVelocity = DegreesPerSecond.of(360);
  private final LinearVelocity maxLinearVelocity = MetersPerSecond.of(4);

  public SwerveSubsystem() {
    Pigeon2 gyro = new Pigeon2(14);

    var fl = createModule(new SparkMax(1, MotorType.kBrushless),
                          new SparkMax(2, MotorType.kBrushless),
                          new CANcoder(3), "frontleft",
                          new Translation2d(Inches.of(12), Inches.of(12)));
    var fr = createModule(new SparkMax(4, MotorType.kBrushless),
                          new SparkMax(5, MotorType.kBrushless),
                          new CANcoder(6), "frontright",
                          new Translation2d(Inches.of(12), Inches.of(-12)));
    var bl = createModule(new SparkMax(7, MotorType.kBrushless),
                          new SparkMax(8, MotorType.kBrushless),
                          new CANcoder(9), "backleft",
                          new Translation2d(Inches.of(-12), Inches.of(12)));
    var br = createModule(new SparkMax(10, MotorType.kBrushless),
                          new SparkMax(11, MotorType.kBrushless),
                          new CANcoder(12), "backright",
                          new Translation2d(Inches.of(-12), Inches.of(-12)));

    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro.getYaw().asSupplier())
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));

    drive = new SwerveDrive(config);
    SmartDashboard.putData("Field", field);
  }

  private SwerveModule createModule(SparkMax driveMotor, SparkMax azimuthMotor,
                                    CANcoder encoder, String name, Translation2d location) {
    Distance wheelDiameter = Inches.of(4);
    
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(wheelDiameter)
        .withClosedLoopController(0.3, 0, 0)
        .withGearing(new MechanismGearing(12.75))
        .withFeedforward(new SimpleMotorFeedforward(0, 2.7, 0.01))
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);

    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(1, 0, 0)
        .withGearing(new MechanismGearing(6.75))
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);

    SmartMotorController driveSMC = new SparkWrapper(driveMotor, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuthMotor, DCMotor.getNEO(1), azimuthCfg);

    return new SwerveModule(new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(encoder.getAbsolutePosition().asSupplier())
        .withTelemetry(name, TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true));
  }

  public SwerveInputStream getDriveInput(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return new SwerveInputStream(drive, x, y, rot)
        .withMaximumAngularVelocity(maxAngularVelocity)
        .withMaximumLinearVelocity(maxLinearVelocity)
        .withDeadband(0.01)
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl();
  }

  public Command drive(Supplier<ChassisSpeeds> speeds) {
    return run(() -> drive.setFieldRelativeChassisSpeeds(speeds.get()));
  }

  public Command driveToPose(Pose2d pose) { return drive.driveToPose(pose); }
  public Command lock() { return run(drive::lockPose); }
  public Pose2d getPose() { return drive.getPose(); }
  public void resetOdometry(Pose2d pose) { drive.resetOdometry(pose); }

  @Override
  public void periodic() {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
  }
}
```

## Next Steps

- Learn about [AdvantageKit Integration](../details/advantagekit-integration.md) for swerve
- See [Organizing Configs in Constants](../details/config-organization.md) for cleaner code
