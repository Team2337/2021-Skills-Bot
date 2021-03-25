package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class LPathTrajectory extends TrajectoryCommand {

  public LPathTrajectory(SwerveDrivetrain drivetrain) {
    super(
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(Units.feetToMeters(5), Units.feetToMeters(2.5))
        ),
        new Pose2d(Units.feetToMeters(10), Units.feetToMeters(5), new Rotation2d(0)),
        new TrajectoryConfig(
          Units.feetToMeters(Constants.Swerve.MAX_FEET_PER_SECOND / 3),
          Units.feetToMeters(Constants.Swerve.MAX_FEET_PER_SECOND / 3)
        )
      ),
      drivetrain
    );
  }

}
