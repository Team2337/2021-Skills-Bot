package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

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

  // Path Max Velocities and Accelerations
  //BarrelRacing
  // Max Velocity - 4.145
  // Acceleration - 8.2

  //Slalom
  // Max Velocity - 4.145
  // Acceleration - 7.5

  //Bounce
  // Max Velocity - 4.145
  // Acceleration - 8.2

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

  private static final double DRIVETRAIN_TRACK_WIDTH_INCHES = DRIVETRAIN_WIDTH_INCHES - (MODULE_INSET_WIDTH_INCHES * 2);
  private static final double DRIVETRAIN_WHEEL_BASE_INCHES = DRIVETRAIN_LENGTH_INCHES - (MODULE_INSET_HEIGHT_INCHES * 2);

  // /2 since we're measuring from the center - halfway
  private static final double MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES = DRIVETRAIN_TRACK_WIDTH_INCHES / 2;
  private static final double MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES = DRIVETRAIN_WHEEL_BASE_INCHES / 2;

  // Radius to the wheel modules can be thought of as a triangle - width and length are the two sides
  public static final double DRIVETRAIN_RADIUS_INCHES = Math.hypot(MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES, MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES);

  // Robot-specific configuration for our swerve drive algorithm
  public static final class Swerve {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
    //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
      SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

      /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(Units.inchesToMeters(DRIVETRAIN_TRACK_WIDTH_INCHES) / 2.0, Units.inchesToMeters(DRIVETRAIN_WHEEL_BASE_INCHES) / 2.0);
  }

  public static final int PIXY_CHIP_SELECT = 0;
  public static final int PIXY_ANALOG = 4; //MXP pin 3
  public static final int PIXY_DIGITAL = 5;

  /*******************/
  /* --------------- */
  /* --- CAN IDs --- */
  /* --------------- */
  /*******************/

  public static final int MODULE0_DRIVE_MOTOR_ID = 0;
  public static final int MODULE0_ANGLE_MOTOR_ID = 4;
  public static final int MODULE0_ANGLE_CANCODER_ID = 1;
  public static final double MODULE0_ANGLE_OFFSET = -Math.toRadians(50.701904296875);


  public static final int MODULE1_DRIVE_MOTOR_ID = 1;
  public static final int MODULE1_ANGLE_MOTOR_ID = 5;
  public static final int MODULE1_ANGLE_CANCODER_ID = 2;
  public static final double MODULE1_ANGLE_OFFSET = -Math.toRadians(128.58123779296875);


  public static final int MODULE2_DRIVE_MOTOR_ID = 14;
  public static final int MODULE2_ANGLE_MOTOR_ID = 10;
  public static final int MODULE2_ANGLE_CANCODER_ID = 3;
  public static final double MODULE2_ANGLE_OFFSET = -Math.toRadians(346.63238525390625);


  public static final int MODULE3_DRIVE_MOTOR_ID = 15;
  public static final int MODULE3_ANGLE_MOTOR_ID = 11;
  public static final int MODULE3_ANGLE_CANCODER_ID = 4;
  public static final double MODULE3_ANGLE_OFFSET = -Math.toRadians(286.42730712890625);


  public static final int INTAKE = 6;
}
