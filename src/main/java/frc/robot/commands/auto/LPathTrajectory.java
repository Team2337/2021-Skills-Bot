package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class LPathTrajectory extends TrajectoryCommand {

  public LPathTrajectory(SwerveDrivetrain drivetrain) {
    super(
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(4, 6, new Rotation2d(0)),
        List.of(
          new Translation2d(14.450170714423756, 15-8.504477227340075),
          new Translation2d(12.035366874959738, 15-12.887650583005863),
          new Translation2d(9.640855504734908, 15-8.950911550602331),
          new Translation2d(16.455677080186113, 15-6.5),
          new Translation2d(22.668620756297106, 15-4.344521033305417),
          new Translation2d(20.2132319783547, 15-0.6309991625330155),
          new Translation2d(16.5, 15-4.5),
          new Translation2d(20.25, 15-9.5),
          new Translation2d(25.21149314157955, 15-12.0),
          new Translation2d(27.800169194513263, 15-8.9),
          new Translation2d(23.15577980542631, 15-6.0),
          new Translation2d(5.0, 15-5.927333633962507)
        ),
        new Pose2d(4, 6, new Rotation2d(0)),
        new TrajectoryConfig(
          Units.feetToMeters(13.4),
          Units.feetToMeters(26)
        ).setEndVelocity(Units.feetToMeters(13.4))//.setKinematics(kinematics)//.addConstraint(new CentripetalAccelerationConstraint(6.5)
      ),
      drivetrain
    );
  }

}
