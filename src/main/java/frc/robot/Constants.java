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

    public final class Swerve {
        public static final double GEARRATIO = 8.31;
        public static final double WHEELDIAMETER = 4;
        public static final double TICKSPERREVOLUTION = GEARRATIO * 4096;
        public static final double INCHESPERREVOLUTION = WHEELDIAMETER * Math.PI;
        public static final double TICKSPERINCH = 1550;
        public static final double INCHESPERDEGREE = 0.2722;
        public static final double MINVELOCITY = 1;
        public static final double TICKSPERDEGREE = 102;
        public final static double SLOWROTATESPEED = 0.05;
        public final static boolean SWERVEDEBUG = false;
        public final static boolean ANALOGENCODER = false;
    }

    /*******************/
    /* --------------- */
    /* --- CAN IDs --- */
    /* --------------- */
    /*******************/

    public static int MODULE0_DRIVE_MOTOR_ID = 0;
    public static int MODULE0_ANGLE_MOTOR_ID = 4;

    public static int MODULE1_DRIVE_MOTOR_ID = 1;
    public static int MODULE1_ANGLE_MOTOR_ID = 5;

    public static int MODULE2_DRIVE_MOTOR_ID = 14;
    public static int MODULE2_ANGLE_MOTOR_ID = 10;

    public static int MODULE3_ANGLE_MOTOR_ID = 11;
    public static int MODULE3_DRIVE_MOTOR_ID = 15;

    /**
     * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
     * constants.  This class should not be used for any other purpose.  All constants should be
     * declared globally (i.e. public static final).  Do not put anything functional in this class.
     *
     * <p>It is advised to static finalally import this class (or one of its inner classes) wherever the
     * constants are needed, to reduce verbosity.
     */
    public Constants() {
        if(Robot.isComp) {
            MODULE0_DRIVE_MOTOR_ID = 0;
            MODULE0_ANGLE_MOTOR_ID = 4;

            MODULE1_DRIVE_MOTOR_ID = 1;
            MODULE1_ANGLE_MOTOR_ID = 5;

            MODULE2_DRIVE_MOTOR_ID = 14;
            MODULE2_ANGLE_MOTOR_ID = 10;

            MODULE3_DRIVE_MOTOR_ID = 15;
            MODULE3_ANGLE_MOTOR_ID = 11;
        } else {
            MODULE0_DRIVE_MOTOR_ID = 1;
            MODULE0_ANGLE_MOTOR_ID = 2;

            MODULE1_DRIVE_MOTOR_ID = 3;
            MODULE1_ANGLE_MOTOR_ID = 4;

            MODULE2_DRIVE_MOTOR_ID = 5;
            MODULE2_ANGLE_MOTOR_ID = 6;

            MODULE3_DRIVE_MOTOR_ID = 7;
            MODULE3_ANGLE_MOTOR_ID = 8;
        }
    }
}
