package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
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
 * @author Zach O., Madison J., and Zayd A.
 * @category SWERVE
 */
public class SwerveDrivetrain extends SubsystemBase {

  private Pigeon pigeon;

  /**
   * Array for swerve module Analog sensors, sorted by AnalogInput ports
   * 0 is Front Right
   * 1 is Front Left
   * 2 is Back Left
   * 3 is Back Right
   */
  /**
   * 17x17in robot - wheel centers are 3.25in from each edge = 5.25 from centers
   * Positive x values represent moving toward the front of the robot whereas
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(Units.inchesToMeters(5.25), Units.inchesToMeters(-5.25)),
    new Translation2d(Units.inchesToMeters(5.25), Units.inchesToMeters(5.25)),
    new Translation2d(Units.inchesToMeters(-5.25), Units.inchesToMeters(5.25)),
    new Translation2d(Units.inchesToMeters(-5.25), Units.inchesToMeters(-5.25))
  );

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

    CANAngleSensors = new CANCoder[] {
      new CANCoder(1),
      new CANCoder(2),
      new CANCoder(3),
      new CANCoder(4)
    };

    /* --- Array for modules --- */
    swerveModules = new FXSwerveModule[] {
      new FXSwerveModule(0, new TalonFX(Constants.MODULE0_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE0_ANGLE_MOTOR_ID), 0, CANAngleSensors[0]), // Module 0
      new FXSwerveModule(1, new TalonFX(Constants.MODULE1_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE1_ANGLE_MOTOR_ID), 0, CANAngleSensors[1]), // Module 1
      new FXSwerveModule(2, new TalonFX(Constants.MODULE2_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE2_ANGLE_MOTOR_ID), 0, CANAngleSensors[2]), // Module 2
      new FXSwerveModule(3, new TalonFX(Constants.MODULE3_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE3_ANGLE_MOTOR_ID), 0, CANAngleSensors[3])  // Module 3
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
    double maxSpeedFtPerSec = 13.6;
    double vxFtPerSec = maxSpeedFtPerSec * forward;
    /**
     * Kinematics expects left to be a positive value/right to be a negative value
     * On our Xbox controller - right is a positive value/left is a negative value
     * We need to negate the values
     */
    double vyFtPerSec = maxSpeedFtPerSec * -strafe;
    /**
     * Max speed in feet per second * 12 = inches per second
     * 2pi * radius of the chassis (8.5in) = inches in one revolution
     * inches per second / inches in one revolution =  revolutions per second
     * revolutions per second * 360 = degrees per second
     */
    double maxDegreesPerSec = 1100;
    double omegaDegreesPerSec = maxDegreesPerSec * -rotation;

    // Kinematics expects meters/sec + radians
    double vxMetersPerSecond = Units.feetToMeters(vxFtPerSec);
    double vyMetersPerSecond = Units.feetToMeters(vyFtPerSec);
    double omegaRadiansPerSecond = Units.degreesToRadians(omegaDegreesPerSec);

    ChassisSpeeds chassisSpeeds;
    if (getFieldOriented()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vxMetersPerSecond,
        vyMetersPerSecond,
        omegaRadiansPerSecond,
        Rotation2d.fromDegrees(pigeon.getYaw())
      );
    } else {
      chassisSpeeds = new ChassisSpeeds(
        vxMetersPerSecond,
        vyMetersPerSecond,
        omegaRadiansPerSecond
      );
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    for (int i = 0; i < moduleStates.length; i++) {
      FXSwerveModule module = swerveModules[i];
      SwerveModuleState moduleState = moduleStates[i];
      moduleState = SwerveModuleState.optimize(moduleState, Rotation2d.fromDegrees(module.getCANCoderDegrees()));
      module.setModuleState(moduleState, maxSpeedFtPerSec);
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
      ((getModule(i).getRadians()) %(2 * Math.PI)) * 180 / Math.PI);
    }
    for(int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("Radians/" + i,
      (getModule(i).getRadians()));
    }
    for(int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("Actual Module Angle/" + i, getModule(i).getNormalizedAnalogVoltageRadians());
      SmartDashboard.putNumber("Angle Motor Temperature/" + i, getModule(i).getAngleMotorTemperature());
      SmartDashboard.putNumber("Drive Motor Temperature/" + i, getModule(i).getDriveMotorTemperature());
    }
  }
}
