package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.TrajectoryPathweaverCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchPathweaverBlueA extends TrajectoryPathweaverCommand {
  public GalacticSearchPathweaverBlueA(SwerveDrivetrain drivetrain) throws IOException {
    super("output/Bounce.wpilib.json", drivetrain);
  }
}