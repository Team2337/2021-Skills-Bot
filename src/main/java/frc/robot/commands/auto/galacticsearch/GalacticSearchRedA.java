package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.TrajectoryPathweaverCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchRedA extends TrajectoryPathweaverCommand {
  public GalacticSearchRedA(SwerveDrivetrain drivetrain) throws IOException {
    super("output/Red A9.wpilib.json", drivetrain);
  }
}
