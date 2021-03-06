package frc.robot.commands.auto.autonav;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class Slalom extends PathweaverTrajectoryCommand {
  public Slalom(SwerveDrivetrain drivetrain) throws IOException {
    super("output/SlalomP10.wpilib.json", drivetrain);
  }
}
