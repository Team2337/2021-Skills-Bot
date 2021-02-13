package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class LPathCommand extends SwerveControllerCommand {

  public LPathCommand(SwerveDrivetrain drivetrain) {
/**
 *  Trajectory trajectory,
    Supplier<Pose2d> pose,
    SwerveDriveKinematics kinematics,
    PIDController xController,
    PIDController yController,
    ProfiledPIDController thetaController,
    Supplier<Rotation2d> desiredRotation,
    Consumer<SwerveModuleState[]> outputModuleStates,
    Subsystem... requirements) 
 */
    Trajectory trajectory;

    super(trajectory, drivetrain::getPose, arg2, arg3, arg4, arg5, arg6, arg7, drivetrain);
  }

}
