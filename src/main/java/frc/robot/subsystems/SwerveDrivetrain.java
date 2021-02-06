package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.nerdyfiles.swerve.*;

/**
 * Subsystem where swerve modules are configured,
 * and the calculations from the joystick inputs is handled.
 * Field orientation is set here as well
 *
 * @author Bryce G.
 * @category SWERVE
 */
public class SwerveDrivetrain extends SubsystemBase {

  private Pigeon pigeon;

  // Sets the distances from module to module
  public static final double WHEELBASE = 22.5;
  public static final double TRACKWIDTH = 23.5;

  // Length and width of the robot
  public static final double WIDTH = 29;
  public static final double LENGTH = 30;

  /* --- Private Double Values --- */
  private double deadband = 0.1;
  private double lastAngle[] = new double[4];
  private double total;
  private double average;
  private double iteration;

  /**
   * Offsets the current gyro position to allow for
   * rotational adjustments during the match
   */
  private double gyroOffset = 0;

  /**
   * Array for module angle offsets
   * 0 is Front Right,
   * 1 is Front Left,
   * 2 is Back Left,
   * 3 is Back Right
   */
  private double angleOffsets[];

  /* --- Private Boolean Values --- */
  private boolean isFieldOriented = true;
  private boolean swerveDebug = false;

  /**
   * Array for swerve module Analog sensors, sorted by AnalogInput ports
	 * 0 is Front Right,
	 * 1 is Front Left,
	 * 2 is Back Left,
	 * 3 is Back Right
	 */
  private CANCoder CANAngleSensors[];

  /**
   * Array for swerve module objects, sorted by ID
	 * 0 is Front Right,
	 * 1 is Front Left,
	 * 2 is Back Left,
	 * 3 is Back Right
	 */
  private FXSwerveModule[] swerveModules;

  /* --- Public Double Values --- */
  public double lastRotation;
  public double fieldOrientedAngle;

  /**
   * Subsystem where swerve modules are configured,
   * and the calculations from the joystick inputs is handled.
   * Field orientation is set here as well
   */
  public SwerveDrivetrain(Pigeon pigeon) {
    this.pigeon = pigeon;

    /**
     * If the CAN encoders are used then the angle offsets will equal 0 because they are already configured
     * in the encoder
     */
    if(Constants.Swerve.ANALOGENCODER) {


      if(Robot.isComp) {
        angleOffsets = new double[] {
          /* -0.407217 + Math.PI,   // Module 0
          2.2618739 + Math.PI,   // Module 1
          -1.193802 + Math.PI,   // Module 2
          -0.746431 - Math.PI / 2   // Module 3  */
          4.6603704,//4.6599803,//4.6524127,
          4.034184285652252 + Math.PI, // 3.8954300 //3.8795141 //4.02862
          2.0822764483757705 + Math.PI, // 2.0473980 //2.0331746
          5.89055019742 //6.211150//0.3951469//0.3950974
        };
      } else {
        angleOffsets = new double[] {
          4.5611,  // Module 0 //4.57
          1.278353,   // Module 1 //1.3
          -0.666697, // Module 2 //-0.678327
          -5.90436  // Module 3 -5.95
        };
     }
  } else {
    angleOffsets = new double[] {
      0,  // Module 0
      0,   // Module 1
      0, // Module 2
      0  // Module 3
    };
  }

    CANAngleSensors = new CANCoder[] {
      new CANCoder(1),
      new CANCoder(2),
      new CANCoder(3),
      new CANCoder(4)
    };

    /* --- Array for modules --- */
    swerveModules = new FXSwerveModule[] {
      new FXSwerveModule(0, new TalonFX(Constants.MODULE0_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE0_ANGLE_MOTOR_ID), angleOffsets[0], CANAngleSensors[0]), // Module 0
      new FXSwerveModule(1, new TalonFX(Constants.MODULE1_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE1_ANGLE_MOTOR_ID), angleOffsets[1], CANAngleSensors[1]), // Module 1
      new FXSwerveModule(2, new TalonFX(Constants.MODULE2_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE2_ANGLE_MOTOR_ID), angleOffsets[2], CANAngleSensors[2]), // Module 2
      new FXSwerveModule(3, new TalonFX(Constants.MODULE3_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE3_ANGLE_MOTOR_ID), angleOffsets[3], CANAngleSensors[3])  // Module 3
    };

    // Setup for drive motor inversion (They may not need to be inverted)
    // (True: invered | False: not inverted)
    swerveModules[0].setDriveInverted(false);
    swerveModules[1].setDriveInverted(false);
    swerveModules[2].setDriveInverted(false);
    swerveModules[3].setDriveInverted(false);

  }

  /**
   * Calculates the desired angle of each module,
   * and the speed and direction of the drive motors based on
   * joystick inputs
   * @param forward - double joystick value from the Y axis on the left hand stick
   * @param strafe - double joystick value from the X axis on the left hand stick
   * @param rotation - double joystick value from the X axis on the right hand stick
   */
  public void calculateJoystickInput(double forward, double strafe, double rotation) {

    // Adjusts forward and strafe based on the gyro if set in field oriented mode
    if (getFieldOriented()) {
      double angleRad = Math.toRadians(-pigeon.getYaw()) % (2*Math.PI);
      double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
      strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
      forward = temp;
    }

    /*
     * a -> d adds the rotational value to the robot, then adjusts for the dimensions of the robot
     */
    double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
    double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
    double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
    double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

    /*
     * Calculations to decide the angle value of each module in RADIANS
     * Takes the arctangent of the values to find the angle that the joystick is
     * currently facing, compensating for the dimensions of the robot and rotation
     * as given in the previous step
     */
    double[] angles = new double[]{
      Units.radiansToDegrees(Math.atan2(b, c)),
      Units.radiansToDegrees(Math.atan2(b, d)),
      Units.radiansToDegrees(Math.atan2(a, d)),
      Units.radiansToDegrees(Math.atan2(a, c))
    };

    /*
     * Calculations to decide the drive motor speed value to set to each module
     */
    double[] speeds = new double[]{
      Math.sqrt(b * b + c * c),
      Math.sqrt(b * b + d * d),
      Math.sqrt(a * a + d * d),
      Math.sqrt(a * a + c * c)
    };

    /**
     * Max speed of the drive motor
     */
    double max = speeds[0];
    // sets the max speed
    for (double speed : speeds) {
        if (speed > max) {
            max = speed;
        }

    }

    // Goes through and sets the desired angle and drive speed of each module
    for(int i=0; i<4; i++) {
      if(max > 1) {
        speeds[i] = speeds[i]/max;
      }

      // Sets the angles and speeds if a joystick is beyond zero,
      // otherwise drive stops and the modules are sent to their last angle
      if(
        Math.abs(forward) > deadband
        || Math.abs(strafe) > deadband
        || Math.abs(rotation) > deadband
      ) {
        if(Math.abs((lastAngle[i] - angles[i])) < 90) {
          lastAngle[i] = angles[i];
        }
        getModule(i).setModuleAngle(angles[i]);
        getModule(i).setDriveSpeed(speeds[i]);
      } else {
        getModule(i).setModuleAngle(lastAngle[i]);
        getModule(i).setDriveSpeed(0);
      }
    }
  }

  /**
   * Stops all of the drive motors on each module
   */
  public void stopDriveMotors() {
    for(int i=0; i<4; i++) {
      getModule(i).setDriveSpeed(0);
    }
  }

  /**
   * Stops all of the angle motors on each module
   */
  public void stopAngleMotors() {
    for(int i=0; i<4; i++) {
      getModule(i).setAngleMotorSpeed(0);
    }
  }

  /**
   * Gets the yaw of the gyro in RADIANS, moding it to get in terms of (-2PI -> 2PI)
   * @return - double value in RADIANS (-2PI -> 2PI)
   */
  public double getYaw() {
    return Math.toRadians(pigeon.getYaw()) % (2 * Math.PI);
  }

  /**
   * Gets the module number specified (0 - 3)
   * @param module - int value specifying the module number (0 - 3).
   * <b>NOTE:</b> The module number is not the CAN IDs of the motors.
   * It is the actual module object's ID
   * @return - FXSwerveModule object
   */
  public FXSwerveModule getModule(int module) {
    return swerveModules[module];
  }

  /**
   * Sets the field orientation mode of the robot
   * (True: robot is field oriented | False: robot is robot oriented)
   * @param isFieldOriented - boolean value to set fieldOrientation mode
   */
  public void setFieldOriented(boolean isFieldOriented) {
    this.isFieldOriented = isFieldOriented;
  }

  /**
   * Sets the offset angle for each of the modules
   * @param angle - The angle we want to offset in radians
   */
  public void setGyroOffsetAngle(double angle) {
    this.gyroOffset = angle;
  }

  /**
   * Gets the offset angle for each of the modules
   * @return - Gyro angle offset in radians
   */
  public double getGyroOffsetAngle() {
    return gyroOffset;
  }

  /**
   * Only used in robot periodic to reset the angle offsets
   * @param module - The module number we are reading
   * @return - The average encoder value over 200 iterations
   */
  public double getAverageAnalogValueInRadians(int module) {
    if(iteration < 200) {
      total += getModule(module).getNormalizedAnalogVoltageRadians();
      iteration++;
    }
    average = total / iteration;
   System.out.println("Average" + average);
    return average;
  }

  /**
   * Gets the current field orientation mode
   * (True: robot is field oriented | False: robot is robot oriented)
   * @return - boolean value of current field orientation mode
   */
  public boolean getFieldOriented() {
      return this.isFieldOriented;
  }

  /**
   * Sets the drive encoders for each module
   */
  public void setAllModuleDriveEncoders(int position) {
    // Goes through 4 times and sets the drive encoders
    for(int i = 0; i < 4; i++) {
      getModule(i).setDriveEncoder(position);
    }
  }

  /**
   * Zeros all of the drive encoders
   */
  public void zeroAllDriveEncoders() {
    setAllModuleDriveEncoders(0);
  }



  @Override
  public void periodic() {
    for(int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("ModuleAngle/" + i,
      ((getModule(i).getRadians() - angleOffsets[i]) %(2 * Math.PI)) * 180 / Math.PI);
    }
    for(int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("Radians/" + i,
      (getModule(i).getRadians()));
    }
  if (swerveDebug) {

  }
  for(int i = 0; i < 4; i++) {
    SmartDashboard.putNumber("Actual Module Angle/" + i, getModule(i).getNormalizedAnalogVoltageRadians());
      SmartDashboard.putNumber("Angle Motor Temperature/" + i, getModule(i).getAngleMotorTemperature());
      SmartDashboard.putNumber("Drive Motor Temperature/" + i, getModule(i).getDriveMotorTemperature());
    }
  }
}
