package frc.robot.subsystems;

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
  private double sentDegree;
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
      new FXSwerveModule(Constants.Swerve.ModulePosition.FRONT_RIGHT, new TalonFX(Constants.MODULE0_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE0_ANGLE_MOTOR_ID), new CANCoder(Constants.MODULE0_ANGLE_CANCODER_ID), Constants.MODULE0_ANGLE_OFFSET),
      new FXSwerveModule(Constants.Swerve.ModulePosition.FRONT_LEFT, new TalonFX(Constants.MODULE1_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE1_ANGLE_MOTOR_ID), new CANCoder(Constants.MODULE1_ANGLE_CANCODER_ID), Constants.MODULE1_ANGLE_OFFSET),
      new FXSwerveModule(Constants.Swerve.ModulePosition.BACK_LEFT, new TalonFX(Constants.MODULE2_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE2_ANGLE_MOTOR_ID), new CANCoder(Constants.MODULE2_ANGLE_CANCODER_ID), Constants.MODULE2_ANGLE_OFFSET),
      new FXSwerveModule(Constants.Swerve.ModulePosition.BACK_RIGHT, new TalonFX(Constants.MODULE3_DRIVE_MOTOR_ID), new TalonFX(Constants.MODULE3_ANGLE_MOTOR_ID), new CANCoder(Constants.MODULE3_ANGLE_CANCODER_ID), Constants.MODULE3_ANGLE_OFFSET)
    };
  }

  public void resetPosition(Pose2d pose) {
    odometry.resetPosition(pose, Rotation2d.fromDegrees(pigeon.getYaw()));
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public double getDriveMotorPosition(int module) {
    return modules[module].getDriveMotorPosition();
  }

  public double getTicksInches(double inches) {
        return modules[0].getTicksFromInches(inches);
}

  /**
   * Calculates the desired angle of each module, and the speed and direction of
   * the drive motors based on joystick inputs
   *
   * @param forward         - double joystick value from the Y axis on the left
   *                        hand stick
   * @param strafe          - double joystick value from the X axis on the left
   *                        hand stick
   * @param rotation        - double joystick value from the X axis on the right
   *                        hand stick
   * @param isFieldOriented - If the robot should drive relative to the field
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
    //TODO: Update comment if it works
    double vyFeetPerSecond = Constants.Swerve.MAX_FEET_PER_SECOND * strafe;
    double omegaDegreesPerSecond = Constants.Swerve.MAX_DEGREES_PER_SECOND * rotation;

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

  public void resetOdometry() {
    odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
  }

  public void setModuleStates(SwerveModuleState[] states, boolean shouldUpdateAngle) {
    setModuleStates(states, shouldUpdateAngle, false);
  }

  public void setModuleStates(SwerveModuleState[] states, boolean shouldUpdateAngle, boolean isJoystickControl) {
    for (int i = 0; i < states.length; i++) {
      FXSwerveModule module = modules[i];
      SwerveModuleState moduleState = states[i];
      module.setDesiredState(moduleState, shouldUpdateAngle, isJoystickControl);
    }
  }

  public void setMotionMagic(Rotation2d angleDegrees, double distanceFeet) {
    for (int i = 0; i < modules.length; i++) {
      FXSwerveModule module = modules[i];
      module.setMotionMagic(angleDegrees, distanceFeet);
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

  public void playNote() {
    for(FXSwerveModule module : modules) {
      module.playNote();
    }
  }

  public void stopNote() {
    for(FXSwerveModule module : modules) {
      module.stopNote();
    }
  }

  public void resetDriveMotors() {
    for(FXSwerveModule module : modules) {
      module.resetDriveMotor();
    }
  }

  public void resetAngleMotors() {
    for(FXSwerveModule module : modules) {
      module.resetAngleMotor();
    }
  }


  public void setAngleMotorsTeleop(double degrees) {
    sentDegree = degrees;
    for (int i = 0; i < 2; i++) {
      FXSwerveModule module = modules[i];
      module.setAngle(Rotation2d.fromDegrees(degrees));    
    }
    for (int i = 2; i < 4; i++) {
      FXSwerveModule module = modules[i];
      module.setAngle(Rotation2d.fromDegrees(-degrees));    
    }

  }
  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(pigeon.getYaw()), modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());

    ChassisSpeeds speeds = kinematics.toChassisSpeeds(modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
    SmartDashboard.putNumber("Velocity X", Units.metersToFeet(speeds.vxMetersPerSecond));
    SmartDashboard.putNumber("Velocity Y", Units.metersToFeet(speeds.vyMetersPerSecond));
    SmartDashboard.putNumber("PoseYaw", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("input angle", sentDegree);

    for(FXSwerveModule module : modules) {
      module.logDebug();
    }
  }

}
