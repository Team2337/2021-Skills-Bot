package frc.robot.commands.auto;

import java.io.IOException;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveTrajectoryCommand extends TrajectoryCommand {

  public SwerveTrajectoryCommand(String path, SwerveDrivetrain drivetrain) throws IOException {
    super(
      generateTrajectory(path),
      false,
      1,
      drivetrain
    );
  }

  private static Trajectory generateTrajectory(String path) throws IOException {
    var trajectory = TrajectoryUtil.fromPathweaverJson(
      Filesystem.getDeployDirectory().toPath().resolve(path)
    );
    var states = trajectory.getStates();
    var start = trajectory.getInitialPose();
    var end = states.get(states.size() - 1).poseMeters;
    var interiorWaypoints = states.subList(1, states.size() - 1).stream().map(s -> s.poseMeters).collect(Collectors.toList());
    return TrajectoryGenerator.generateTrajectory(
      start,
      interiorWaypoints,
      end,
      new TrajectoryConfig(
        4.125,
        8.2
      ).setEndVelocity(4.125)
    );
  }

}
