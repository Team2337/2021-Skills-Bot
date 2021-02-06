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

    public static final class Auton {
        public static final double INCHESTOJOYSTICKVALUE = 1;
        public static final double AUTOSTRAFEP = 1.25;
    }

    public final class KickerWheel {
        public final static double SHORTVELOCITYP = 0.0000525; 
    }
    
    /*******************/
    /* --------------- */
    /* --- CAN IDs --- */
    /* --------------- */
    /*******************/

    public static int CANID0;
    public static int MODULE0DRIVEMOTORID = 0;
    public static int MODULE1DRIVEMOTORID = 1;
    public static int SHOOTERRIGHTMOTOR = 2;
    public static int SHOOTERLEFTMOTOR = 3;
    public static int MODULE0ANGLEMOTORID = 4;
    public static int MODULE1ANGLEMOTORID = 5;
    public static int KICKER = 6;
                      //Limelight = 7
    public static int INTAKE = 8;
    public static int AGITATOR = 9;
    public static int MODULE2ANGLEMOTORID = 10;
    public static int MODULE3ANGLEMOTORID = 11;
    public static int SERIALIZER = 12;
    public static int CLIMBER = 13;
    public static int MODULE2DRIVEMOTORID = 14;
    public static int MODULE3DRIVEMOTORID = 15;

    /***************/
    /* ----------- */
    /* --- PCM --- */
    /* ----------- */
    /***************/

    public static final int PCM0 = 0;
    public static final int PCM1 = 1;

    /*********************/
    /* ----------------- */
    /* --- PCM PORTS --- */
    /* ----------------- */
    /*********************/

    public static int PCM0PORT0 = 0;
    public static int PCM0PORT1 = 1;
    public static int PCM0PORT2 = 2;
    public static int PCM0PORT3 = 3;
    public static int PCM0PORT4 = 4;
    public static int PCMLEDSTRIP = 5;
    public static int PCM0PORT6 = 6;
    public static int PCM0PORT7 = 7;

    /*********************/
    /* ----------------- */
    /* --- DIO PORTS --- */
    /* ----------------- */
    /*********************/

    public static int DIOPORT0 = 0;
    public static int DIOPORT1 = 1;
    public static int DIOPORT2 = 2;
    public static int DIOPORT3 = 3;
    public static int DIOPORT4 = 4;
    public static int DIOPORT5 = 5;
    public static int DIOPORT6 = 6;
    public static int DIOPORT7 = 7;
    public static int DIOPORT8 = 8;
    public static int DIOPORT9 = 9;

    /************************/
    /* -------------------- */
    /* --- ANALOG PORTS --- */
    /* -------------------- */
    /************************/

    public static int ANALOGPORT0 = 0;
    public static int ANALOGPORT1 = 1;
    public static int ANALOGPORT2 = 2;
    public static int ANALOGPORT3 = 3;

    /***********************/
    /* ------------------- */
    /* --- RELAY PORTS --- */
    /* ------------------- */
    /***********************/

    public static int RELAYPORT0 = 0;
    public static int RELAYPORT1 = 1;
    public static int RELAYPORT2 = 2;
    public static int RELAYPORT3 = 3;

    /*********************/
    /* ----------------- */
    /* --- PWM PORTS --- */
    /* ----------------- */
    /*********************/

    public static int PWMPORT0 = 0;
    public static int PWMPORT1 = 1;
    public static int PWMPORT2 = 2;
    public static int PWMPORT3 = 3;
    public static int PWMBLINKIN = 4;
    public static int PWMPORT5 = 5;
    public static int PWMPORT6 = 6;
    public static int PWMPORT7 = 7;
    public static int PWMPORT8 = 8;
    public static int SERVOPORT = 9;

    /* --- TIME OF FLIGHT Variables --- */

    /** Configure range mode. 0=short; 1=medium; 2=long */
    public static int TOFMODE = 0;


    /********************/
    /* ---------------- */
    /* --- AGITATOR --- */
    /* ---------------- */
    /********************/

    /** Percent speed of the agitator */
    public static double AGITATORSPEED = 0.4;

    public static double AGITATORSHOOTSPEED = 0.5;

    /** Percent speed of the agitator */
    public static double AGITATORREVERSESPEED = -0.2;

    /******************/
    /* -------------- */
    /* --- INTAKE --- */
    /* -------------- */
    /******************/
    
   /* --- Intake --- */
   public static double INTAKEFORWARDSPEED = 0.5;
   public static double INTAKEREVERSESPEED = -0.4;

   //Whether or not to detect jams for the agitator
   public static boolean DETECTINTAKEJAMS = true;

   //The current to trigger motor reversal at
   public static int INTAKECURRENTTOLERENCE = 40;

   //The amount of time (in seconds) to reverse the Serializer when a jam is detected
   public static double INTAKEREVERSALDURATION = 0.4;


    /*******************/
    /* --------------- */
    /* --- CLIMBER --- */
    /* --------------- */
    /*******************/

    /** Percent speed on the climber */
    public static double CLIMBERSPEED = 0.7;


    /*******************/
    /* --------------- */
    /* --- SHOOTER --- */
    /* --------------- */
    /*******************/

    /**
     * This value is the number at which the closed loop ramp rate of the shooter
     * goes from 0.5 to 0 to increase speed
     */
    public static int SHOOTERRAMPSWITCHVALUE = 5000;
    
    /** Speed to shoot at from ~16 feet away */
    public static int SHOOTSPEEDCLOSE = 13025; 
    /** Speed to shoot at from ~34 feet away */
    public static int SHOOTSPEEDFAR = 15400; 

    public static int SHOOTFRONTTRENCHSPEED = 13750;

    public static int SHOOTFRONTTRENCHAUTO = 13300;

    /**********************/
    /* ------------------ */
    /* --- SERIALIZER --- */
    /* ------------------ */
    /**********************/

    /** Maximum speed of the serializer */
    public static double SERIALIZERPEAKSPEED = 0.4;

    /** Percent speed on the serializer when moving to positions */
    public static double SERIALIZERPOSITIONSPEED = 0.2;

    /** Percent forward speed when serializing or shooting */
    public static double SERIALIZERDRIVERFORWARDSPEED = 0.25;
    
    public static double SERIALIZEROPERATORFORWARDSPEED = 0.3;
    /** Percent reverse speed when serializing or shooting */
    public static double SERIALIZERREVERSESPEED = -0.3;
    
    //Amount of ticks to reverse the serializer by when readying the kicker wheel
    public static double SERIALIZERREGRESSIONDISTANCE = -4096;

    /******************/
    /* -------------- */
    /* --- KICKER --- */
    /* -------------- */
    /******************/
    
    /** Kicker wheel velocity for the far shot */
    public static int KICKERSPEEDFAR;

    /** Kicker wheel velocity for the near shot */
    public static int KICKERSPEEDCLOSE;

    public static int KICKERSPEEDFRONTTRENCH;

    public static double KICKERCONTROLPANELSPEED = 12000;

    /******************/
    /* -------------- */
    /* --- VISION --- */
    /* -------------- */
    /******************/

    /** P value for vision rotation */
    public static double VISIONCLOSEROTATIONP = 2.5; 
    public static double VISIONMIDDLEROTATIONP = 0.9; 
    public static double VISIONOFFROTATIONP = 0.65; 
    public static double VISIONFARROTATIONP = 0.85; 

    public static double BALLTRACKINGP = 0.65;


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
            MODULE0DRIVEMOTORID = 0;
            MODULE1DRIVEMOTORID = 1;
            SHOOTERRIGHTMOTOR = 2;
            SHOOTERLEFTMOTOR = 3;
            MODULE0ANGLEMOTORID = 4;
            MODULE1ANGLEMOTORID = 5;
            KICKER = 6;
            //Limelight = 7
            INTAKE = 8;
            AGITATOR = 9;
            MODULE2ANGLEMOTORID = 10;
            MODULE3ANGLEMOTORID = 11;
            SERIALIZER = 12;
            CLIMBER = 13;
            MODULE2DRIVEMOTORID = 14;
            MODULE3DRIVEMOTORID = 15;

            /* --- Kicker --- */
            KICKERSPEEDCLOSE = 7000; 
            KICKERSPEEDFAR = 5000; 
            KICKERSPEEDFRONTTRENCH = 4000;

        } else {
            CANID0 = 0;
            MODULE0DRIVEMOTORID = 1;
            MODULE0ANGLEMOTORID = 2;
            MODULE1DRIVEMOTORID = 3;
            MODULE1ANGLEMOTORID = 4;
            MODULE2DRIVEMOTORID = 5;
            MODULE2ANGLEMOTORID = 6;
            MODULE3DRIVEMOTORID = 7;
            MODULE3ANGLEMOTORID = 8;
            INTAKE = 9;
            AGITATOR = 10;
            CLIMBER = 11;
            KICKER = 12;
            SHOOTERLEFTMOTOR = 13;
            SHOOTERRIGHTMOTOR = 14;
            SERIALIZER = 15;

            /* --- Shooter --- */
            SHOOTSPEEDCLOSE = 12500;
            SHOOTSPEEDFAR = 14800;
            SHOOTFRONTTRENCHSPEED = 13700;
            SHOOTFRONTTRENCHAUTO = 13300;
            
            /* --- Kicker --- */
            KICKERSPEEDCLOSE = 3000 * (7 / 3);
            KICKERSPEEDFAR = 4500 * (7/3);
            KICKERSPEEDFRONTTRENCH = 4500 * (7/3);

        }
    }
}
