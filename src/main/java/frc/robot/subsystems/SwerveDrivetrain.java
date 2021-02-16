package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.swerve.*;

/**
 * Subsystem where swerve modules are configured,
 * and the calculations from the joystick inputs is handled.
 * Field orientation is set here as well
 *
 * @author Madison J. and Zayd A.
 * @category SWERVE
 */
public class SwerveDrivetrain extends SubsystemBase {

  private Pigeon pigeon;

  /**
   * Array for swerve module objects, sorted by ID
   * 0 is Front Right,
   * 1 is Front Left,
   * 2 is Back Left,
   * 3 is Back Right
   */
  private FXSwerveModule[] modules;

  /**
   * Should be in the same order as the swerve modules (see above)
   * Positive x values represent moving toward the front of the robot
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(
      Units.inchesToMeters(Constants.Swerve.MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES),
      Units.inchesToMeters(-Constants.Swerve.MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(Constants.Swerve.MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES),
      Units.inchesToMeters(Constants.Swerve.MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(-Constants.Swerve.MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES),
      Units.inchesToMeters(Constants.Swerve.MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(-Constants.Swerve.MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES),
      Units.inchesToMeters(-Constants.Swerve.MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES)
    )
  );

  private SwerveDriveOdometry odometry;

  /**
   * Subsystem where swerve modules are configured,
   * and the calculations from the joystick inputs is handled.
   * Field orientation is set here as well
   */
  public SwerveDrivetrain(Pigeon pigeon) {
    this.pigeon = pigeon;

    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(pigeon.getYaw()));

    SmartDashboard.putNumber("ticks", 0);

    modules = new FXSwerveModule[] {
      new FXSwerveModule(0, new TalonFX(Constants.MODULE0_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE0_ANGLE_MOTOR_ID), new CANCoder(Constants.MODULE0_ANGLE_CANCODER_ID), 0), // Module 0
      new FXSwerveModule(1, new TalonFX(Constants.MODULE1_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE1_ANGLE_MOTOR_ID), new CANCoder(Constants.MODULE1_ANGLE_CANCODER_ID), 0), // Module 1
      new FXSwerveModule(2, new TalonFX(Constants.MODULE2_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE2_ANGLE_MOTOR_ID), new CANCoder(Constants.MODULE2_ANGLE_CANCODER_ID), 0), // Module 2
      new FXSwerveModule(3, new TalonFX(Constants.MODULE3_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE3_ANGLE_MOTOR_ID), new CANCoder(Constants.MODULE3_ANGLE_CANCODER_ID), 0)  // Module 3
    };
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Calculates 
   * he desired angle of each  
   * and the speed  
   * nd direction o   the drive motors based on
   * joystick inputs
   * @
   *                 aram forward - double joy   tick value from the Y axis on the left hand stick
   * 
   *                 param strafe - double joystick value from the X axis on the left hand stick
   * @pa
   *                 am rotation - double joystick value from the X axis on the right hand stick
   */
  public void calculateJoystickInput(double forward, double strafe, double rotation, boolean isFieldOriented) {
    // By default, our angle motors will reset back to 0
    // If we let go of our joysticks, we don't want our angle motors to snap to a position
    // We want to stay still, so the robot does not adjust once we've stopped moving
    boolean shouldUpdateAngle = true;
    if (forward == 0 && strafe == 0 && rotation == 0) {
      shouldUpdateAngle = false;
    }

    double vxFeetPerSeccond = Constants.Swerve.MAX_FEET_PER_SECOND * forward;
    /**
     * For our joystick X axes - kinematics expects left to be a positive value,
     * right to be a negative value. On our Xbox controller - right is a positive value,
     * left is a negative value. We need to negate the values to work with kinematics.
     */
    double vyFeetPerSecond = Constants.Swerve.MAX_FEET_PER_SECOND * -strafe;
    double omegaDegreesPerSecond = Constants.Swerve.MAX_DEGREES_PER_SECOND * -rotation;

    // Kinematics expects meters/sec + radians
    double vxMetersPerSecond = Units.feetToMeters(vxFeetPerSeccond);
    double vyMetersPerSecond = Units.feetToMeters(vyFeetPerSecond);
    double omegaRadiansPerSecond = Units.degreesToRadians(omegaDegreesPerSecond);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      vxMetersPerSecond,
      vyMetersPerSecond,
      omegaRadiansPerSecond
    );

    if (isFieldOriented) {
      // If the robot is field-oriented, use a field-oriented field speed instead
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vxMetersPerSecond,
        vyMetersPerSecond,
        omegaRadiansPerSecond,
        Rotation2d.fromDegrees(pigeon.getYaw())
      );
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates, shouldUpdateAngle);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states, true);
  }

  public void setModuleStates(SwerveModuleState[] states, boolean shouldUpdateAngle) {
    for (int i = 0; i < states.length; i++) {
      FXSwerveModule module = modules[i];
      SwerveModuleState moduleState = states[i];
      module.setDesiredState(moduleState, shouldUpdateAngle);
    }
  }

  /**
   * Stops all of the drive motors on each module
   */
  public void stopDriveMotors() {
    for(FXSwerveModule module : modules) {
      module.stopDriveMotor();
    }
  }

  /**
   * Stops all of the angle motors on each module
   */
  public void stopAngleMotors() {
    for(FXSwerveModule module : modules) {
      module.stopAngleMotor();
    }
  }

  /**
   * Sets all modules turn motors to a specific set point tick
   */
  public void setTurnMotorTicks() {
    double tick = SmartDashboard.getNumber("ticks", 0);
    for(FXSwerveModule module : modules) {
      module.angleMotor.set(ControlMode.Position, tick);
    }
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(pigeon.getYaw()), modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());

    for(FXSwerveModule module : modules) {
      SmartDashboard.putNumber("Module Angle (Degrees)/" + module.getModuleNumber(), module.getAngle());
      SmartDashboard.putNumber("Angle Motor Temperature/" + module.getModuleNumber(), module.getAngleMotorTemperature());
      SmartDashboard.putNumber("Drive Motor Temperature/" + module.getModuleNumber(), module.getDriveMotorTemperature());
    }
  }

}
