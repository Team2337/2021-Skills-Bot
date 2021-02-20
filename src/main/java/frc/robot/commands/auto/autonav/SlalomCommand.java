package frc.robot.commands.auto.autonav;

import java.io.IOException;

import frc.robot.commands.auto.TrajectoryPathweaverCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class SlalomCommand extends TrajectoryPathweaverCommand {
  public SlalomCommand(SwerveDrivetrain drivetrain) throws IOException {
    super("output/Slalom.wpilib.json", drivetrain);
  }
}
