package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.TrajectoryPathweaverCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchRedB extends TrajectoryPathweaverCommand {
  public GalacticSearchRedB(SwerveDrivetrain drivetrain) throws IOException {
    super("output/Red B3.wpilib.json", drivetrain);
  }
}
