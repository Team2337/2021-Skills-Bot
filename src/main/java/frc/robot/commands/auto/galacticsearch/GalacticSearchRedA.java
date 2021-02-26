package frc.robot.commands.auto.galacticsearch;

import java.util.List;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.commands.auto.TrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchRedA extends TrajectoryCommand {
  
  public GalacticSearchRedA(SwerveDrivetrain drivetrain){
    super(
      TrajectoryGenerator.generateTrajectory(
        List.of(),
        Constants.SWERVE_TRAJECTORY_CONFIG
      ),
      drivetrain
    );
  }
}