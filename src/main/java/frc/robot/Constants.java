package frc.robot;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static final).  Do not put anything functional in this class.
 *
 * <p>It is advised to static finalally import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * 17x17in robot - since the values are the same, we'll only define one value
   * as opposed to having a length and a width. Keep in mind - this will not work
   * in the future if the robot is not a perfect square.
   */
  private static final double DRIVETRAIN_WIDTH_INCHES = 17;
  public static final double DRIVETRAIN_LENGTH_INCHES = 17;

  // The module inset from the outside edges of the robot
  private static final double MODULE_INSET_WIDTH_INCHES = 3.25;
  private static final double MODULE_INSET_HEIGHT_INCHES = 3.25;

  private static final double TRACK_WIDTH = DRIVETRAIN_WIDTH_INCHES - (MODULE_INSET_WIDTH_INCHES * 2);
  private static final double WHEEL_BASE = DRIVETRAIN_LENGTH_INCHES - (MODULE_INSET_HEIGHT_INCHES * 2);

  //Swerve trajectory config
  public static final TrajectoryConfig SWERVE_TRAJECTORY_CONFIG = new TrajectoryConfig(
    Units.feetToMeters(Constants.Swerve.MAX_FEET_PER_SECOND),
    Units.feetToMeters(Constants.Swerve.MAX_FEET_PER_SECOND)
  );

    // Robot-specific configuration for our swerve drive algorithm
  public static final class Swerve {

    public enum ModulePosition {
      FRONT_RIGHT(0),
      FRONT_LEFT(1),
      BACK_LEFT(2),
      BACK_RIGHT(3);
            
      public final int value;

      ModulePosition(int value) {
        this.value = value;
      }
    }

    // /2 since we're measuring from the center - halfway
    public static final double MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES = TRACK_WIDTH / 2;
    public static final double MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES = WHEEL_BASE / 2;

    // Radius to the wheel modules can be thought of as a triangle - width and length are the two sides
    private static final double DRIVETRAIN_RADIUS_INCHES = Math.hypot(MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES, MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES);

    /**
     * The max unadjusted speed in feet/sec for a Falcon 500 motor with the MK3
     * Swerve Drive Specialties module is 13.6 feet/second
     * https://www.swervedrivespecialties.com/products/mk3-swerve-module
     */
    public static final double MAX_FEET_PER_SECOND = 20; //TODO: 
    private static final double MAX_INCHES_PER_SECOND = MAX_FEET_PER_SECOND * 12;
    /**
     * To calculate max rotational speed:
     * Max speed in feet per second * 12 = inches per second
     * 2pi * radius of the chassis (3.71231 in) = inches in one revolution
     * inches per second / inches in one revolution =  revolutions per second
     * revolutions per second * 360 degrees = degrees per second
     */
    private static final double INCHES_PER_REVOLUTION = Math.PI * 2 * DRIVETRAIN_RADIUS_INCHES;
    private static final double REVOLUTION_PER_SECOND = MAX_INCHES_PER_SECOND / INCHES_PER_REVOLUTION;
    public static final double MAX_DEGREES_PER_SECOND = REVOLUTION_PER_SECOND * 360;
  }

  public static final int PIXY_CHIP_SELECT = 0;

  /*******************/
  /* --------------- */
  /* --- CAN IDs --- */
  /* --------------- */
  /*******************/

  public static final int MODULE0_DRIVE_MOTOR_ID = 0;
  public static final int MODULE0_ANGLE_MOTOR_ID = 4;
  public static final int MODULE0_ANGLE_CANCODER_ID = 1;
  public static final double MODULE0_ANGLE_OFFSET = -230.801;

  public static final int MODULE1_DRIVE_MOTOR_ID = 1;
  public static final int MODULE1_ANGLE_MOTOR_ID = 5;
  public static final int MODULE1_ANGLE_CANCODER_ID = 2;
  public static final double MODULE1_ANGLE_OFFSET = -36.826;

  public static final int MODULE2_DRIVE_MOTOR_ID = 14;
  public static final int MODULE2_ANGLE_MOTOR_ID = 10;
  public static final int MODULE2_ANGLE_CANCODER_ID = 3;
  public static final double MODULE2_ANGLE_OFFSET = -346.992;

  public static final int MODULE3_DRIVE_MOTOR_ID = 15;
  public static final int MODULE3_ANGLE_MOTOR_ID = 11;
  public static final int MODULE3_ANGLE_CANCODER_ID = 4;
  public static final double MODULE3_ANGLE_OFFSET = -15.557;

  public static final int INTAKE = 6;
}
