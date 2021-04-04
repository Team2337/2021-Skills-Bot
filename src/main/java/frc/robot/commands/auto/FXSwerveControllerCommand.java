package frc.robot.commands.auto;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
 * {@link Trajectory} with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an array. The desired wheel and module
 * rotation velocities should be taken from those and used in velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes to the angle given in the final state of the trajectory.
 */
@SuppressWarnings("MemberName")
public class FXSwerveControllerCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;
  private final Supplier<Rotation2d> m_desiredRotation;
  private Boolean m_shouldUpdate;

  /**
   * Constructs a new FXSwerveControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but rather
   * raw module states from the position controllers which need to be put into a
   * velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the outputVolts to zero upon completion
   * of the path- this is left to the user, since it is not appropriate for paths
   * with nonstationary endstates.
   *
   * @param trajectory         The trajectory to follow.
   * @param pose               A function that supplies the robot pose - use one
   *                           of the odometry classes to provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param xController        The Trajectory Tracker PID controller for the
   *                           robot's x position.
   * @param yController        The Trajectory Tracker PID controller for the
   *                           robot's y position.
   * @param thetaController    The Trajectory Tracker PID controller for angle for
   *                           the robot.
   * @param desiredRotation    The angle that the drivetrain should be facing.
   *                           This is sampled at each time step.
   * @param outputModuleStates The raw output module states from the position
   *                           controllers.
   * @param requirements       The subsystems to require.
   */
  @SuppressWarnings("ParameterName")
  public FXSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
      PIDController xController, PIDController yController, ProfiledPIDController thetaController,
      Optional<Supplier<Rotation2d>> desiredRotation, Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "FXSwerveControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "FXSwerveControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "FXSwerveControllerCommand");

    // Either -180 to 180 or 0 to 360 - trying the former, might need to try the latter
    thetaController.enableContinuousInput(-(Math.PI), (Math.PI));
    m_controller = new HolonomicDriveController(
        requireNonNullParam(xController, "xController", "FXSwerveControllerCommand"),
        requireNonNullParam(yController, "xController", "FXSwerveControllerCommand"),
        requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand"));

    m_outputModuleStates = requireNonNullParam(outputModuleStates, "frontLeftOutput", "SwerveControllerCommand");

    if (desiredRotation.isEmpty()) {
      m_desiredRotation = () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
    } else {
      m_desiredRotation = requireNonNullParam(desiredRotation.get(), "desiredRotation", "SwerveControllerCommand");
    }

    m_shouldUpdate = false;

    addRequirements(requirements);
  }

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but rather
   * raw module states from the position controllers which need to be put into a
   * velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the outputVolts to zero upon completion
   * of the path- this is left to the user, since it is not appropriate for paths
   * with nonstationary endstates.
   *
   * <p>
   * Note 2: The final rotation of the robot will be set to the rotation of the
   * final pose in the trajectory. The robot will not follow the rotations from
   * the poses at each timestep. If alternate rotation behavior is desired, the
   * other constructor with a supplier for rotation should be used.
   *
   * @param trajectory         The trajectory to follow.
   * @param pose               A function that supplies the robot pose - use one
   *                           of the odometry classes to provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param xController        The Trajectory Tracker PID controller for the
   *                           robot's x position.
   * @param yController        The Trajectory Tracker PID controller for the
   *                           robot's y position.
   * @param thetaController    The Trajectory Tracker PID controller for angle for
   *                           the robot.
   * @param outputModuleStates The raw output module states from the position
   *                           controllers.
   * @param requirements       The subsystems to require.
   */
  @SuppressWarnings("ParameterName")
  public FXSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
      PIDController xController, PIDController yController, ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates, Subsystem... requirements) {
      this(
          trajectory,
          pose,
          kinematics,
          xController,
          yController,
          thetaController,
          Optional.empty(),
          outputModuleStates,
          requirements
      );
  }

  public FXSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
  PIDController xController, PIDController yController, ProfiledPIDController thetaController, Optional<Supplier<Rotation2d>> desiredRotation,
  Boolean shouldUpdate, Consumer<SwerveModuleState[]> outputModuleStates, Subsystem... requirements) {
    this(
        trajectory,
        pose,
        kinematics,
        xController,
        yController,
        thetaController,
        desiredRotation,
        outputModuleStates,
        requirements
    );
    m_shouldUpdate = shouldUpdate;
  }

  public FXSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
  PIDController xController, PIDController yController, ProfiledPIDController thetaController,
  Boolean shouldUpdate, Consumer<SwerveModuleState[]> outputModuleStates, Subsystem... requirements) {
    this(
        trajectory,
        pose,
        kinematics,
        xController,
        yController,
        thetaController,
        Optional.empty(),
        outputModuleStates,
        requirements
    );
    m_shouldUpdate = shouldUpdate;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = m_timer.get();
    Rotation2d m_actualRotation = m_desiredRotation.get();

    var desiredState = m_trajectory.sample(curTime);

    var pose = m_pose.get();

    if (m_shouldUpdate){
      m_actualRotation = desiredState.poseMeters.getRotation();
    }
    SmartDashboard.putBoolean("shouldUpdate", m_shouldUpdate);

    var targetChassisSpeeds = m_controller.calculate(pose, desiredState, m_actualRotation);
    SmartDashboard.putString("targetChassisSpeeds", targetChassisSpeeds.toString());

    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    SmartDashboard.putNumber("pathX", desiredState.poseMeters.getTranslation().getX());
    SmartDashboard.putNumber("pathHeading", desiredState.poseMeters.getRotation().getDegrees());
    SmartDashboard.putNumber("pathY", desiredState.poseMeters.getTranslation().getY());
    SmartDashboard.putNumber("robotX", pose.getX());
    SmartDashboard.putNumber("robotHeading", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("robotY", pose.getY());
    SmartDashboard.putNumber("curTime", curTime);
    SmartDashboard.putNumber("desiredRotation", m_actualRotation.getDegrees());

    m_outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
