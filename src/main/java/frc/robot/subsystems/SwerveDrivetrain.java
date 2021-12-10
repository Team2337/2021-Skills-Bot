package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper.GearRatio;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem where swerve modules are configured,
 * and the calculations from the joystick inputs is handled.
 * Field orientation is set here as well
 *
 * @author Madison J. and Zayd A.
 * @category SWERVE
 */
public class SwerveDrivetrain extends SubsystemBase {

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private Pigeon pigeon;

  /**
   * Array for swerve module objects, sorted by ID
   * 0 is Front Right,
   * 1 is Front Left,
   * 2 is Back Left,
   * 3 is Back Right
   */
  private SwerveModule[] modules;

  /**
   * Should be in the same order as the swerve modules (see above)
   * Positive x values represent moving toward the front of the robot
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES)
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
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    modules = new SwerveModule[] {
      Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(0, 0),
        GearRatio.STANDARD,
        Constants.MODULE0_DRIVE_MOTOR_ID,
        Constants.MODULE0_ANGLE_MOTOR_ID,
        Constants.MODULE0_ANGLE_CANCODER_ID,
        Constants.MODULE0_ANGLE_OFFSET
      ),
      Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(2, 0),
        GearRatio.STANDARD,
        Constants.MODULE1_DRIVE_MOTOR_ID,
        Constants.MODULE1_ANGLE_MOTOR_ID,
        Constants.MODULE1_ANGLE_CANCODER_ID,
        Constants.MODULE1_ANGLE_OFFSET
      ),
      Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(4, 0),
        GearRatio.STANDARD,
        Constants.MODULE2_DRIVE_MOTOR_ID,
        Constants.MODULE2_ANGLE_MOTOR_ID,
        Constants.MODULE2_ANGLE_CANCODER_ID,
        Constants.MODULE2_ANGLE_OFFSET
      ),
      Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(6, 0),
        GearRatio.STANDARD,
        Constants.MODULE3_DRIVE_MOTOR_ID,
        Constants.MODULE3_ANGLE_MOTOR_ID,
        Constants.MODULE3_ANGLE_CANCODER_ID,
        Constants.MODULE3_ANGLE_OFFSET
      )
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

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(pigeon.getFusedHeading());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  public void resetOdometry() {
    odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
  }

  /**
   * Stops all of the motors on each module
   */
  public void stopMotors() {
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    this.chassisSpeeds = kinematics.toChassisSpeeds(states);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < states.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState moduleState = states[i];
      module.set(moduleState.speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * Constants.Swerve.MAX_VOLTAGE, moduleState.angle.getRadians());
    }

    odometry.update(
      Rotation2d.fromDegrees(pigeon.getYaw()),
      states[0],
      states[1],
      states[2],
      states[3]
    );
  }

}
