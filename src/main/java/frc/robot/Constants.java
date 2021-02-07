package frc.robot;

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
    public static final double DRIVETRAIN_LENGTH_INCHES = 17;

    // Robot-specific configuration for our swerve drive algorithm
    public final class Swerve {
        // The module inset from the outside edges of the robot
        public static final double MODULE_INSET_INCHES = 3.25;
        // /2 since we're measuring from the center - halfway
        public static final double MODULE_DISTANCE_FROM_CENTER_INCHES = (DRIVETRAIN_LENGTH_INCHES / 2) - MODULE_INSET_INCHES;
        // Since the robot is a square the radius is simple - be careful when adapting for other bases
        private static final double DRIVETRAIN_RADIUS_INCHES = (DRIVETRAIN_LENGTH_INCHES / 2);

        /**
         * The max unadjusted speed in feet/sec for a Falcon 500 motor with the
         * MK3 Swerve Drive Specialties module is 13.6 feet/second
         * https://www.swervedrivespecialties.com/products/mk3-swerve-module
         */
        public static final double MAX_FEET_PER_SECOND = 13.6;
        private static final double MAX_INCHES_PER_SECOND = MAX_FEET_PER_SECOND * 12;
        /**
         * To calculate max rotational speed:
         * Max speed in feet per second * 12 = inches per second
         * 2pi * radius of the chassis (8.5in) = inches in one revolution
         * inches per second / inches in one revolution =  revolutions per second
         * revolutions per second * 360 degrees = degrees per second
         */
        private static final double INCHES_PER_REVOLUTION = Math.PI * 2 * DRIVETRAIN_RADIUS_INCHES;
        private static final double REVOLUTION_PER_SECOND = MAX_INCHES_PER_SECOND / INCHES_PER_REVOLUTION;
        public static final double MAX_DEGREES_PER_SECOND = REVOLUTION_PER_SECOND * 360;
    }

    /*******************/
    /* --------------- */
    /* --- CAN IDs --- */
    /* --------------- */
    /*******************/

    public static final int MODULE0_DRIVE_MOTOR_ID = 0;
    public static final int MODULE0_ANGLE_MOTOR_ID = 4;
    public static final int MODULE0_ANGLE_CANCODER_ID = 1;

    public static final int MODULE1_DRIVE_MOTOR_ID = 1;
    public static final int MODULE1_ANGLE_MOTOR_ID = 5;
    public static final int MODULE1_ANGLE_CANCODER_ID = 2;

    public static final int MODULE2_DRIVE_MOTOR_ID = 14;
    public static final int MODULE2_ANGLE_MOTOR_ID = 10;
    public static final int MODULE2_ANGLE_CANCODER_ID = 3;

    public static final int MODULE3_ANGLE_MOTOR_ID = 11;
    public static final int MODULE3_DRIVE_MOTOR_ID = 15;
    public static final int MODULE3_ANGLE_CANCODER_ID = 4;
}
