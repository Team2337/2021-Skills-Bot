package frc.robot.commands.auto.autonav;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class Slalom extends PathweaverTrajectoryCommand {

  public Slalom(Optional<Supplier<Rotation2d>> desiredRotation, SwerveDrivetrain drivetrain) throws IOException {
    super("output/SlalomP3.wpilib.json", false, 1, drivetrain);
  }

  public Slalom(SwerveDrivetrain drivetrain) throws IOException {
    this(Optional.empty(), drivetrain);
  }

  public Slalom(Supplier<Rotation2d> desiredRotation, SwerveDrivetrain swerveDrivetrain) throws IOException {
    this(Optional.of(desiredRotation), swerveDrivetrain);
  }

}
