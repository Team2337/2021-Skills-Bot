package frc.robot.commands.auto.galacticsearch;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class GalacticSearchRedA extends PathweaverTrajectoryCommand {

  public GalacticSearchRedA(SwerveDrivetrain drivetrain) throws IOException {
    // super("output/RedA_favorred.wpilib.json", true, 7, drivetrain);
    super("output/RedA_favorredAndle.wpilib.json", true, 7, drivetrain);
  }

}
