package frc.robot;

/**
 * Utilities class to house basic methods that can be used across subsystems
 * @author Bryce G.
 */
public class Utilities {

    /**
     * Checks to see if the absolute value of the input is less than the deadband
     * @param input - Value in which the deadband will be applied (0 < input < 1)
     * @param deadband - Deadband to set on the input (double)
     * @return - input double value adjusted for the deadband
     */
    public static double deadband(double input, double deadband) {
      if (Math.abs(input) < deadband) return 0;
		  return Math.copySign((Math.abs(input) - deadband) / (1 - deadband), input);
    }

    /**
     * Squares the input value
     * @param value - double value wanting to be squared
     * @return - squared input double value
     */
    public static double squareValues(double value) {
      // double direction = value < 0 ? -1 : 1;
      return Math.copySign(Math.pow(value, 2), value);
    }

    /**
     * Calculates derivative based on the current and previous sensor inputs
     * @param error - double value reading the current sensor error (current - target)
     * @param lastError - double value reading the previous sensor error (current - target)
     * @param dt - Change in time from when the previous sensor error was gotten to the time the current was gotten
     * @return - returns derivative double value to add to the speed of the motor
     */
    public static double calculateDerivative(double error, double lastError, double dt) {
      if (Double.isFinite(lastError)) {
        return (error - lastError) / dt;
      } else {
        return 0;
      }
    }


  /**
   * Determines whether or not the given value is within a certain amount of a target
   *
   * @param target The desired value
   * @param current The current value
   * @param tolerance A range that the given value can be within the target value before returning true
   * @return Whether or not the current value is within a tolerance of the target
   */
  public static boolean withinTolerance(double target, double current, double tolerance) {
    return Math.abs(target - current) <= tolerance;
  }

  public static double scaleRotationP(double input) {
    double rotationP = (0.5/input) * 0.5;
    return rotationP;
  }

  /**
 * Scales an input to a new output
 * 
 * @param input Value to be translated to new range
 * @param oldmin Minimum value of the old range
 * @param oldmax Maximum value of the new range
 * @param newmin Minimum value of the new range
 * @param newmax Maximum value of the new range
 * @return
 */
public static double scaleToRange(double input, double oldmin, double oldmax, double newmin, double newmax) {
    
  double oldrange = oldmax - oldmin;
  double newrange = newmax - newmin;
  double newoutput = (((input -oldmin)*newrange)/ oldrange) + newmin;
  return newoutput;
}


}