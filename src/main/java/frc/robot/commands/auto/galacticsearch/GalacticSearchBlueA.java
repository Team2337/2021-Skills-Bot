package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.TrajectoryPathweaverCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchBlueA extends TrajectoryPathweaverCommand {
  public GalacticSearchBlueA(SwerveDrivetrain drivetrain) throws IOException {
    super("output/Blue A6.wpilib.json", drivetrain);
  }
}
