package frc.robot.commands.auto.autonav;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class BarrelRacing extends PathweaverTrajectoryCommand {

  public BarrelRacing(Optional<Supplier<Rotation2d>> desiredRotation, SwerveDrivetrain drivetrain) throws IOException {
    super("output/BarrelRacingV2RB.wpilib.json", desiredRotation, false, 10, drivetrain);
  }

  public BarrelRacing(SwerveDrivetrain drivetrain) throws IOException {
    this(Optional.empty(), drivetrain);
  }

  public BarrelRacing(Supplier<Rotation2d> desiredRotation, SwerveDrivetrain swerveDrivetrain) throws IOException {
    this(Optional.of(desiredRotation), swerveDrivetrain);
  }

}
