package frc.robot.commands.auto.autonav;

import java.io.IOException;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.commands.auto.TrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class Slalom2 extends TrajectoryCommand {

  public Slalom2(SwerveDrivetrain drivetrain) throws IOException {
    super(
      TrajectoryGenerator.generateTrajectory(
        TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("output/SlalomP4.wpilib.json")).getStates().stream().map(s -> s.poseMeters).collect(Collectors.toList()),
        new TrajectoryConfig(
          4.145,
          6.5
        )
        .addConstraint(new CentripetalAccelerationConstraint(4.5))
      ),
      drivetrain
    );

  }
}
