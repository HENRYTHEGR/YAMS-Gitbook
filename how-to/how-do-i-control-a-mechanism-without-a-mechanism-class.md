---
description: >-
  Sometimes you have "loosely mechanically coupled" design which requires you to
  control both motors independently instead of using one as a follower.
---

# How do I control a Mechanism without a Mechanism Class?

## Subsystems with `SmartMotorController`'s

Subsystems given when you create the `SmartMotorControllerConfig` are only used for YAMS generated commands. This means that if you're using a YAMS generated command like `SmartMotorController.sysId()` or `Arm.run` or `Elevator.run` that command will be the **ONLY** command allowed to run on that subsystems.&#x20;

## Controlling an Elevator without Elevator.run()

IF you know how to use Command Based Programming well enough you can create your own commands and control the `SmartMotorController` without the Mechanism but remember to create the Mechanism and Mechanism config because it will modify and re-apply the `SmartMotorControllerConfig` &#x20;

The following example shows exactly how you can recreate the commands used in YAMS without using the helpful functions to create them in the first place and could allow you to use multiple mechanisms in the same Subsystem if you needed it. (For example a shooter and indexer)

An example of this for an Elevator would be as follows

<pre class="language-java"><code class="lang-java">public class ExampleSubsystem extends SubsystemBase {
  
  // Chain driven elevator drum calculation
  private Distance chainPitch = Inches.of(0.25);
  private int teethCount = 22;
  private Distance drumCircumference = chainPitch.times(teethCount);
  private Distance drumRadius = drumCircumference.div(2*Math.PI);
  
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  private MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(3,4));
  
  // Profiled PID Controller
  private double kP = 4, kI = 0, kD = 0;
  private LinearVelocity maxVel = MetersPerSecond.of(0.5);
  private LinearAcceleration maxAccel = MetersPerSecondPerSecond.of(0.5);
  
  // Feedforwards
  private ElevatorFeedforward realFeedforward = new ElevatorFeedforward(0,0,0);
  private ElevatorFeedforward simFeedforward = new ElevatorFeedforward(0,0,0);
  
  private MotorMode idle = MotorMode.BRAKE;
  private Current statorLimit = Amps.of(40);
  // Ramp rate is how long the motor should take to run from 0 to 100%
  private Time openLoopRampRate = Seconds.of(0.25);
  private Time closedLoopRampRate = Seconds.of(0.25);
  
  private SmartMotorControllerConfig leftSmcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Elevator drum radius: derived from chain pitch and tooth count.
  .withDrumRadius(drumRadius)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(kP, kI, kD)
  .withTrapezoidalProfile(maxVel, maxAccel)
  // Uncomment below to set sim only values.
  //.withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  // Feedforward Constants
  .withFeedforward(realFeedforward)
  .withSimFeedforward(simFeedforward)
  // Telemetry name and verbosity level
  .withTelemetry("LeftElevatorMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(gearing)
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(idle)
  .withStatorCurrentLimit(statorLimit)
  .withClosedLoopRampRate(closedLoopRampRate)
  .withOpenLoopRampRate(openLoopRampRate);
  
  private SmartMotorControllerConfig rightSmcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Elevator drum radius: derived from chain pitch and tooth count.
  .withDrumRadius(drumRadius)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(kP, kI, kD)
  .withTrapezoidalProfile(maxVel, maxAccel)
  // Uncomment below to set sim only values.
  //.withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  // Feedforward Constants
  .withFeedforward(realFeedforward)
  .withSimFeedforward(simFeedforward)
  // Telemetry name and verbosity level
  .withTelemetry("RightElevatorMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(gearing)
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(idle)
  .withStatorCurrentLimit(statorLimit)
  .withClosedLoopRampRate(closedLoopRampRate)
  .withOpenLoopRampRate(openLoopRampRate);

  // Vendor motor controller object
  private SparkMax leftSpark = new SparkMax(4, MotorType.kBrushless);
  private SparkMax rightSpark = new SparkMax(6, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController leftSparkSmartMotorController = new SparkWrapper(leftSpark, DCMotor.getNEO(1), leftSmcConfig);
  private SmartMotorController rightSparkSmartMotorController = new SparkWrapper(rightSpark, DCMotor.getNEO(1), rightSmcConfig);
  
  // Include this only if you want a pretty sim
  /*
  private ElevatorConfig elevconfig = new ElevatorConfig()
      .withStartingHeight(Meters.of(0.5))
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withCarriageWeight(Pounds.of(16));

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig, leftSparkSmartMotorController);
  */
  
  /**
   * Set the dutycycle of the SmartMotorController without interrupting 
   *
   * @param dutyCycle Value between [-1, 1] which will control the speed of the motor.
   * @return StartRun command which turns off the closed loop controller, sets the dutycycle of the SMC then turns the closed loop controller back on when interrupted.
   */
<strong>  public Command setDutyCycle(double dutyCycle)
</strong><strong>  {
</strong><strong>    return startRun(()->{leftSparkSmartMotorController.stopClosedLoopController();rightSparkSmartMotorController.stopClosedLoopController();}, // Stop the closed loop controller since the motor is in ControlMode.CLOSED_LOOP
</strong><strong>           () -> {leftSparkSmartMotorController.setDutyCycle(dutycycle);rightSparkSmartMotorController.setDutyCycle(dutycycle);}) // Apply the dutycycle given
</strong><strong>                   .finallyDo(()->{leftSparkSmartMotorController.startClosedLoopController();rightSparkSmartMotorController.startClosedLoopController();}) // Start the closed loop controller when this command is interrupted
</strong><strong>                   .withName("Custom SetDutyCycle"); // Be nice, give your command name :)
</strong><strong>  }
</strong>  
  /**
   * Set the position of the spark motor controller.
   * @param position Mechanism position to go to.
   * @return RunCommand which will set the position of the mechanism until interrupted.
   */
<strong>  public Command setMechanismPosition(Angle position)
</strong><strong>  {
</strong><strong>    return run(() -> {leftSparkSmartMotorController.setPosition(position);rightSparkSmartMotorController.setPosition(position);})
</strong><strong>            .withName("Custom SetPosition");
</strong><strong>  }
</strong>  
  /**
   * Set the position of the spark motor controller.
   * @param position Measurement position to go to.
   * @return RunCommand which will set the position of the measurement until interrupted.
   */
<strong>  public Command setMechanismPosition(Distance height)
</strong><strong>  {
</strong><strong>    return run(() -> {leftSparkSmartMotorController.setPosition(position);rightSparkSmartMotorController.setPosition(position);})
</strong><strong>            .withName("Custom SetPosition with Height");
</strong><strong>  }
</strong>  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftSparkSmartMotorController.updateTelemetry();
    rightSparkSmartMotorController.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    leftSparkSmartMotorController.simIterate();
    rightSparkSmartMotorController.simIterate();
  }
}
</code></pre>

{% hint style="success" %}
Remember that mechanisms will re-apply a modified `SmartMotorControllerConfig` with changes from their respective config classes when the mechanism is created!&#x20;

So even if you're not going to use the mechanism you should still create it!
{% endhint %}
