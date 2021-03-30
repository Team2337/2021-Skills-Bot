package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class TrajectoryCommand extends FXSwerveControllerCommand {

  private Trajectory trajectory;
  private SwerveDrivetrain drivetrain;

  public TrajectoryCommand(Trajectory trajectory, Boolean shouldUpdate, double thetaP, SwerveDrivetrain drivetrain) {
    // Note on thetaP: set to 11 for Galatic Search, set to 1 for AutoNav, except Barrel Racing which is 10
    // There was also a comment around thetaP being 10 for Bounce, but I can't confirm that (~zach)
    // We should look to bring this thetaP down for Galactic Search, since we're over-correcting in the videos
    super(
      trajectory,
      drivetrain::getPose,
      drivetrain.getKinematics(),
      new PIDController(5.5, 0, 0),
      new PIDController(3, 0, 0),
      new ProfiledPIDController(thetaP, 0, 0, new TrapezoidProfile.Constraints(
        Units.degreesToRadians(Constants.Swerve.MAX_DEGREES_PER_SECOND),
        Units.degreesToRadians(Constants.Swerve.MAX_DEGREES_PER_SECOND)
      )),
      shouldUpdate,
      drivetrain::setModuleStates,
      drivetrain
    );

    this.trajectory = trajectory;
    this.drivetrain = drivetrain;
  }


  public TrajectoryCommand(Trajectory trajectory, SwerveDrivetrain drivetrain) {
    this(trajectory, false, 1, drivetrain);
  }

  @Override
  public void initialize() {
    super.initialize();

    drivetrain.resetPosition(trajectory.getInitialPose());
  }

}
