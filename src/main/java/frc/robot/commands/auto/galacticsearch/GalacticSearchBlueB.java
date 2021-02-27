package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.TrajectoryPathweaverCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchBlueB extends TrajectoryPathweaverCommand {
  public GalacticSearchBlueB(SwerveDrivetrain drivetrain) throws IOException {
    super("output/Blue B1.wpilib.json", drivetrain);
  }
}
