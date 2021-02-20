package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class TrajectoryCommand extends SwerveControllerCommand {

  public TrajectoryCommand(Trajectory trajectory, SwerveDrivetrain drivetrain) {
    // TODO: We know our velocity is correct, we need to figure out if our acceleration is correct
    super(
      trajectory,
      drivetrain::getPose,
      drivetrain.getKinematics(),
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(
        Units.degreesToRadians(Constants.Swerve.MAX_DEGREES_PER_SECOND),
        Units.degreesToRadians(Constants.Swerve.MAX_DEGREES_PER_SECOND)
      )),
      drivetrain::setModuleStates,
      drivetrain
    );
  }

}