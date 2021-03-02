package frc.robot.commands.auto.autonav;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class Bounce extends PathweaverTrajectoryCommand {
  public Bounce(SwerveDrivetrain drivetrain) throws IOException {
    super("output/Bounce.wpilib.json", drivetrain);
  }
}
