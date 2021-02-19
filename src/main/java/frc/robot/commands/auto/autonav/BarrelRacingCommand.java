package frc.robot.commands.auto.autonav;

import java.io.IOException;

import frc.robot.commands.auto.TrajectoryPathweaverCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class BarrelRacingCommand extends TrajectoryPathweaverCommand {
  public BarrelRacingCommand(SwerveDrivetrain drivetrain) throws IOException {
    super("output/BarrelRacing.wpilib.json", drivetrain);
  }
}
