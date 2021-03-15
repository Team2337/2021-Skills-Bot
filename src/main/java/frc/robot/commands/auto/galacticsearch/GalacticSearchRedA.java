package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchRedA extends PathweaverTrajectoryCommand {
  public GalacticSearchRedA(SwerveDrivetrain drivetrain) throws IOException {
    super("output/RedA.wpilib.json", drivetrain);
  }
}
