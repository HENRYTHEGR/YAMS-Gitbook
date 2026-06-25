---
description: We will learn how to program a Swerve Drive!
---

# Swerve Drive

{% hint style="danger" %}
Swerve drive requires more hardware configuration than a tank drive. Make sure you have installed REVLib (for SparkMax motors) and Phoenix 6 (for CANcoder and Pigeon2) before starting.
{% endhint %}

## Intro

At the end of this tutorial you will have a full swerve drive subsystem that works in both real life and simulation with the same code!

## Details

This swerve drive will be using the following hardware and control details:

* 4x `SparkMax` (NEO) drive motors — CAN IDs 1, 4, 7, 10
* 4x `SparkMax` (NEO) azimuth motors — CAN IDs 2, 5, 8, 11
* 4x `CANcoder` absolute encoders — CAN IDs 3, 6, 9, 12
* `Pigeon2` gyro — CAN ID 14
* SDS MK4i modules: drive gear ratio `12.75:1`, steer gear ratio `6.75:1`, 4-inch wheels
* Left stick controls translation, right stick X controls rotation

{% hint style="warning" %}
**Simulation Processing Requirements**: YAMS simulates **each motor individually**. With 4 modules containing 2 motors each, that is 8 `SmartMotorController` instances running physics simulation simultaneously. This provides highly accurate simulation but requires more processing power than simplified kinematic-only simulations.
{% endhint %}

## Let's make a Swerve Drive!

{% stepper %}
{% step %}
#### Configure the Drive Motor Controllers

We start with the `SmartMotorControllerConfig` for the **drive** motors. The drive motors use velocity control, so we configure a PID controller and a feedforward.

<pre class="language-java" data-title="SwerveSubsystem.java" data-line-numbers data-full-width="true"><code class="lang-java">package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

<strong>import edu.wpi.first.math.controller.SimpleMotorFeedforward;
</strong><strong>import edu.wpi.first.math.geometry.Translation2d;
</strong>import edu.wpi.first.wpilibj2.command.SubsystemBase;
<strong>import yams.gearing.MechanismGearing;
</strong><strong>import yams.motorcontrollers.SmartMotorControllerConfig;
</strong><strong>import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
</strong>
public class SwerveSubsystem extends SubsystemBase {

<strong>  // kP=0.3 trims velocity error left after the feedforward.
</strong><strong>  // kV = 12 V / (max wheel speed in rad/s): at full speed the feedforward
</strong><strong>  // alone commands 12 V, leaving PID to correct only disturbances.
</strong><strong>  private SmartMotorControllerConfig buildDriveCfg() {
</strong><strong>    return new SmartMotorControllerConfig(this)
</strong><strong>        .withWheelDiameter(Inches.of(4))
</strong><strong>        .withClosedLoopController(0.3, 0, 0)
</strong><strong>        .withGearing(new MechanismGearing(12.75))  // MK4i L1 ratio
</strong><strong>        .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / (MetersPerSecond.of(1).in(MetersPerSecond) / Inches.of(4).in(Meters)), 0.01))
</strong><strong>        .withStatorCurrentLimit(Amps.of(40))        // prevents belt slip
</strong><strong>        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
</strong><strong>  }
</strong>
}
</code></pre>

The `withFeedforward` calculates `kV` from your maximum linear velocity and wheel diameter so the feedforward alone handles most of the work.
{% endstep %}

{% step %}
#### Configure the Azimuth Motor Controllers

The **azimuth** (steering) motors use position control. A single `kP=1` is typically enough at a `6.75:1` reduction because the gear ratio makes the output stiff.

<pre class="language-java" data-title="SwerveSubsystem.java" data-line-numbers data-full-width="true"><code class="lang-java">package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig buildDriveCfg() {
    return new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(0.3, 0, 0)
        .withGearing(new MechanismGearing(12.75))
        .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / (MetersPerSecond.of(1).in(MetersPerSecond) / Inches.of(4).in(Meters)), 0.01))
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
  }

<strong>  // kP=1 with a 6.75:1 reduction gives a stiff, responsive steer.
</strong><strong>  // 20 A is enough — the steer motor almost never stalls during normal driving.
</strong><strong>  private SmartMotorControllerConfig buildAzimuthCfg() {
</strong><strong>    return new SmartMotorControllerConfig(this)
</strong><strong>        .withClosedLoopController(1, 0, 0)
</strong><strong>        .withFeedforward(new SimpleMotorFeedforward(0, 1))
</strong><strong>        .withGearing(new MechanismGearing(6.75))   // MK4i steer ratio
</strong><strong>        .withStatorCurrentLimit(Amps.of(20))
</strong><strong>        .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);
</strong><strong>  }
</strong>

}
</code></pre>
{% endstep %}

{% step %}
#### Build a `SwerveModule` Factory

Now we write a helper method that takes the two `SparkMax` objects and a `CANcoder`, wraps them in `SmartMotorController`s, and returns a configured `SwerveModule`.

<pre class="language-java" data-title="SwerveSubsystem.java" data-line-numbers data-full-width="true"><code class="lang-java">package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
<strong>import yams.mechanisms.config.SwerveModuleConfig;
</strong><strong>import yams.mechanisms.swerve.SwerveModule;
</strong><strong>import yams.motorcontrollers.SmartMotorController;
</strong>import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
<strong>import yams.motorcontrollers.local.SparkWrapper;
</strong>

public class SwerveSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig buildDriveCfg() { /* ... same as before ... */ }
  private SmartMotorControllerConfig buildAzimuthCfg() { /* ... same as before ... */ }

<strong>  /**
</strong><strong>   * All four modules share the same gearing and PID gains, so one factory
</strong><strong>   * method creates all of them. Pass each module a unique name and location.
</strong><strong>   */
</strong><strong>  public SwerveModule createModule(SparkMax drive, SparkMax azimuth,
</strong><strong>                                   CANcoder absoluteEncoder, String moduleName,
</strong><strong>                                   Translation2d location) {
</strong><strong>    SmartMotorController driveSMC   = new SparkWrapper(drive,   DCMotor.getNEO(1), buildDriveCfg());
</strong><strong>    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), buildAzimuthCfg());
</strong>
<strong>    return new SwerveModule(new SwerveModuleConfig(driveSMC, azimuthSMC)
</strong><strong>        // CANcoder eliminates the need to home the steer motor at startup.
</strong><strong>        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
</strong><strong>        .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
</strong><strong>        .withLocation(location)
</strong><strong>        // State optimization rotates the module at most 90 deg instead of 180 deg + reversing drive.
</strong><strong>        .withOptimization(true));
</strong><strong>  }
</strong>

}
</code></pre>

The `withLocation(Translation2d)` call uses WPILib's coordinate convention: `+X` is toward the front of the robot, `+Y` is toward the left side.
{% endstep %}

{% step %}
#### Instantiate the Four Modules and Create `SwerveDrive`

In the constructor, we create all four modules and pass them to `SwerveDriveConfig`. The gyro supplier gives field-relative driving its heading reference.

<pre class="language-java" data-title="SwerveSubsystem.java" data-line-numbers data-full-width="true"><code class="lang-java">package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
<strong>import com.ctre.phoenix6.hardware.Pigeon2;
</strong>import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveModuleConfig;
<strong>import yams.mechanisms.config.SwerveDriveConfig;
</strong><strong>import yams.mechanisms.swerve.SwerveDrive;
</strong>import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class SwerveSubsystem extends SubsystemBase {

<strong>  private final SwerveDrive drive;
</strong><strong>  private final Field2d field = new Field2d();
</strong>
  // ... buildDriveCfg(), buildAzimuthCfg(), createModule() from previous steps ...

  public SwerveSubsystem() {
<strong>    // CAN ID 14 — update to match your robot's Pigeon2 CAN ID.
</strong><strong>    Pigeon2 gyro = new Pigeon2(14);
</strong>
<strong>    // Module locations: +X forward, +Y left. 24-inch offsets assume module
</strong><strong>    // centers are 24 in from the robot center — update to match your chassis.
</strong><strong>    // CAN IDs are grouped as (drive, steer, CANcoder) per module.
</strong><strong>    var fl = createModule(new SparkMax(1, MotorType.kBrushless),
</strong><strong>                          new SparkMax(2, MotorType.kBrushless),
</strong><strong>                          new CANcoder(3), "frontleft",
</strong><strong>                          new Translation2d(Inches.of(24), Inches.of(24)));
</strong><strong>    var fr = createModule(new SparkMax(4, MotorType.kBrushless),
</strong><strong>                          new SparkMax(5, MotorType.kBrushless),
</strong><strong>                          new CANcoder(6), "frontright",
</strong><strong>                          new Translation2d(Inches.of(24), Inches.of(-24)));
</strong><strong>    var bl = createModule(new SparkMax(7, MotorType.kBrushless),
</strong><strong>                          new SparkMax(8, MotorType.kBrushless),
</strong><strong>                          new CANcoder(9), "backleft",
</strong><strong>                          new Translation2d(Inches.of(-24), Inches.of(24)));
</strong><strong>    var br = createModule(new SparkMax(10, MotorType.kBrushless),
</strong><strong>                          new SparkMax(11, MotorType.kBrushless),
</strong><strong>                          new CANcoder(12), "backright",
</strong><strong>                          new Translation2d(Inches.of(-24), Inches.of(-24)));
</strong>
<strong>    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
</strong><strong>        // gyro.getYaw() gives the heading used for field-relative driving.
</strong><strong>        .withGyro(gyro.getYaw().asSupplier())
</strong><strong>        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
</strong><strong>        // Translation and rotation PIDs are used by driveToPose(); kP=1 is a conservative start.
</strong><strong>        .withTranslationController(new PIDController(1, 0, 0))
</strong><strong>        .withRotationController(new PIDController(1, 0, 0));
</strong>
<strong>    drive = new SwerveDrive(config);
</strong><strong>    SmartDashboard.putData("Field", field);
</strong>  }

}
</code></pre>
{% endstep %}

{% step %}
#### Add Periodic Updates and Drive Commands

`periodic()` must call `drive.updateTelemetry()` to keep odometry and SmartDashboard fresh. `simulationPeriodic()` must call `drive.simIterate()` to step the physics engine.

<pre class="language-java" data-title="SwerveSubsystem.java" data-line-numbers data-full-width="true"><code class="lang-java">  // ... fields and constructor from previous steps ...

  /**
   * Drive the robot with field-relative chassis speeds.
   */
<strong>  public Command drive(Supplier&#x3C;ChassisSpeeds> speedsSupplier) {
</strong><strong>    return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get()))
</strong><strong>        .withName("Field Oriented Drive");
</strong><strong>  }
</strong>
  /** Drive to a specific field pose. */
<strong>  public Command driveToPose(Pose2d pose) { return drive.driveToPose(pose); }
</strong>
  /** Lock wheels in X pattern to resist pushing. */
<strong>  public Command lock() { return run(drive::lockPose); }
</strong>

  @Override
  public void periodic() {
<strong>    drive.updateTelemetry();           // updates pose estimator and telemetry
</strong><strong>    field.setRobotPose(drive.getPose()); // keeps the Field2d widget current
</strong>  }

  @Override
  public void simulationPeriodic() {
<strong>    drive.simIterate();  // steps the motor physics simulation
</strong>  }
</code></pre>

You can also expose `getPose()`, `resetOdometry()`, and `addVisionMeasurement()` as needed for autonomous and vision integration.
{% endstep %}

{% step %}
#### Create a `SwerveInputStream` and Bind the Default Command

`SwerveInputStream` converts raw joystick values into `ChassisSpeeds` with deadband, cubing, and alliance-relative field orientation applied automatically. Set it as the subsystem's default command in `RobotContainer`.

<pre class="language-java" data-title="SwerveSubsystem.java" data-line-numbers><code class="lang-java">  // Add this method to SwerveSubsystem
<strong>  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationX,
</strong><strong>                                                     DoubleSupplier translationY,
</strong><strong>                                                     DoubleSupplier rotation) {
</strong><strong>    return new SwerveInputStream(drive, translationX, translationY, rotation)
</strong><strong>        .withMaximumAngularVelocity(DegreesPerSecond.of(360))
</strong><strong>        .withMaximumLinearVelocity(MetersPerSecond.of(1))
</strong><strong>        .withDeadband(0.01)
</strong><strong>        .withCubeRotationControllerAxis()     // non-linear rotation response
</strong><strong>        .withCubeTranslationControllerAxis()  // non-linear translation response
</strong><strong>        .withAllianceRelativeControl();        // alliance-aware field-relative
</strong><strong>  }
</strong>
</code></pre>

<pre class="language-java" data-title="RobotContainer.java" data-line-numbers><code class="lang-java">public class RobotContainer {

<strong>  private final SwerveSubsystem drive = new SwerveSubsystem();
</strong><strong>  private final CommandXboxController xboxController = new CommandXboxController(0);
</strong>
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

<strong>    // Left stick translates, right stick X rotates.
</strong><strong>    drive.setDefaultCommand(drive.drive(
</strong><strong>        drive.getChassisSpeedsSupplier(
</strong><strong>            xboxController::getLeftY,
</strong><strong>            xboxController::getLeftX,
</strong><strong>            xboxController::getRightX)));
</strong>
    configureBindings();
  }

  private void configureBindings() {
<strong>    xboxController.button(5).whileTrue(drive.driveToPose(
</strong><strong>        new Pose2d(Meters.of(3), Meters.of(3), Rotation2d.fromDegrees(30))));
</strong>  }
}
</code></pre>
{% endstep %}
{% endstepper %}

## Complete Example

The following files show a complete swerve drive implementation matching the hardware described in this tutorial.

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/swerve_drive/java/frc/robot/subsystems/SwerveSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/swerve_drive/java/frc/robot/RobotContainer.java" %}

## Next Steps

* Learn about [AdvantageKit Integration](../details/advantagekit-integration.md) for swerve
* See [Organizing Configs in Constants](../details/config-organization.md) for cleaner code
