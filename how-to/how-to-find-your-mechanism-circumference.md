# How to find your mechanism circumference?

YAMS converts between rotational encoder counts and real-world linear distance using a mechanism's circumference. Getting this value right is critical: an incorrect circumference causes every position setpoint to be off by a constant factor, making closed-loop control impossible to tune.

<figure><img src="../.gitbook/assets/image.png" alt=""><figcaption></figcaption></figure>

Mechanism circumference is used to convert mechanism rotations to measurements. This is primarily useful for Elevators or Linear Slides where the primary unit of control is in `Distance` (usually meters).

Your **Mechanism Circumference** is the circumference of your final sprocket/gear which translates directly to Distance.

For elevators this is the drum circumference $$c = 2*\pi * r$$ OR if driven by a sprocket $$c = pitch*teeth$$

{% hint style="warning" %}
Remember that you measurement ≠ actual height of the mechanism UNLESS you set the elevator starting height to be relative to the floor AND adjust your limits accordingly
{% endhint %}

## Set the circumference in YAMS

```java
SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Elevator drum radius API: pass the chain pitch and tooth count directly.
  .withDrumRadius(Inches.of(0.25), 22)
  .withMechanismCircumference(Inches.of(4).times(Math.PI)) // 4in * PI
  .withWheelDiameter(Inches.of(4)) // Same as above
  .withWheelRadius(Inches.of(2)) // Same as above
  .withStartingPosition(Meters.of(0));
```
