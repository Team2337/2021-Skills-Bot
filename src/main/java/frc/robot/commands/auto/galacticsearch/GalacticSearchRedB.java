package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchRedB extends PathweaverTrajectoryCommand {
  public GalacticSearchRedB(SwerveDrivetrain drivetrain) throws IOException {
    super("output/RedB_favorred.wpilib.json", true, 7, drivetrain);
  }
}
