package frc.robot.commands.heading;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Heading {

  public Double value;
  public Rotation2d rotation;

  public Heading(Double value, Rotation2d rotation) {
    this.value = value;
    this.rotation = rotation;
  }

  private static double lerp(Double startValue, Double endValue, double t) {
    return startValue + (endValue - startValue) * t;
  }

  private static Rotation2d lerp(Rotation2d startValue, Rotation2d endValue, double t) {
    return startValue.plus(endValue.minus(startValue).times(t));
  }

  Heading interpolate(Heading endValue, double i) {
    // Find the new value value
    final double newV = lerp(value, endValue.value, i);

    // Find the delta value between the current state and the interpolated state.
    final double deltaV = newV - value;

    // If delta time is negative, flip the order of interpolation.
    if (deltaV < 0) {
      return endValue.interpolate(this, 1 - i);
    }

    return new Heading(newV, lerp(rotation, endValue.rotation, i));
  }

}