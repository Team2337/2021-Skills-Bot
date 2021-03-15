package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchBlueB extends PathweaverTrajectoryCommand {
  public GalacticSearchBlueB(SwerveDrivetrain drivetrain) throws IOException {
    super("output/BlueB.wpilib.json", drivetrain);
  }
}
