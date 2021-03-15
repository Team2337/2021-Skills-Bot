package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchBlueA extends PathweaverTrajectoryCommand {
  public GalacticSearchBlueA(SwerveDrivetrain drivetrain) throws IOException {
    super("output/BlueA.wpilib.json", drivetrain);
  }
}
