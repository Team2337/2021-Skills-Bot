package frc.robot.commands.auto.autonav;

import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;
// import java.util.stream.Collectors;
import java.util.Optional;

// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.auto.TrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class Slalom2 extends TrajectoryCommand implements PoseSupplier {

  private static List<Pose2d> poses = List.of(
    new Pose2d(Units.feetToMeters(4.3), Units.feetToMeters(2.5), Rotation2d.fromDegrees(0)),
    new Pose2d(Units.feetToMeters(9.0), Units.feetToMeters(7.2), Rotation2d.fromDegrees(0)),
    new Pose2d(Units.feetToMeters(21.0), Units.feetToMeters(7.4), Rotation2d.fromDegrees(-25)),
    new Pose2d(Units.feetToMeters(25.0), Units.feetToMeters(2.75), Rotation2d.fromDegrees(0)),
    new Pose2d(Units.feetToMeters(29.0), Units.feetToMeters(5.5), Rotation2d.fromDegrees(90)),
    new Pose2d(Units.feetToMeters(25.5), Units.feetToMeters(8.25), Rotation2d.fromDegrees(-125)),
    new Pose2d(Units.feetToMeters(21.0), Units.feetToMeters(3.32), Rotation2d.fromDegrees(-179)),
    new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(2.80), Rotation2d.fromDegrees(-179)),
    new Pose2d(Units.feetToMeters(8.9), Units.feetToMeters(4.0), Rotation2d.fromDegrees(-225)),
    new Pose2d(Units.feetToMeters(6.2), Units.feetToMeters(7.2), Rotation2d.fromDegrees(-225)),
    new Pose2d(Units.feetToMeters(1.41), Units.feetToMeters(10), Rotation2d.fromDegrees(-225)),
    new Pose2d(Units.feetToMeters(4.3), Units.feetToMeters(2.5), Rotation2d.fromDegrees(0))
  );

  public Slalom2(SwerveDrivetrain drivetrain) throws IOException {
    this(
      Optional.empty(),
      drivetrain
    );
  }

  public Slalom2(Optional<Supplier<Rotation2d>> desiredRotation, SwerveDrivetrain drivetrain) throws IOException {
    super(
      TrajectoryGenerator.generateTrajectory(
        poses,
        new TrajectoryConfig(
          2,
          4
        )
        .setKinematics(drivetrain.getKinematics())
      ),
      desiredRotation,
      false,
      7,
      drivetrain
    );
  }

  public Slalom2(Supplier<Rotation2d> desiredRotation, SwerveDrivetrain drivetrain) throws IOException {
    this(
      Optional.of(desiredRotation),
      drivetrain
    );

  }

  @Override
  public List<Pose2d> getPoses() {
    return poses;
  }
  
}
