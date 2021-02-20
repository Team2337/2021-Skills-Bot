package frc.robot.commands.auto.autonav;

import java.io.IOException;

import frc.robot.commands.auto.TrajectoryPathweaverCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class BounceCommand extends TrajectoryPathweaverCommand {
  public BounceCommand(SwerveDrivetrain drivetrain) throws IOException {
    super("output/Bounce.wpilib.json", drivetrain);
  }
}
