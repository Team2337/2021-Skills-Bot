package frc.robot.commands.auto.autonav;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class BarrelRacing extends PathweaverTrajectoryCommand {
  public BarrelRacing(SwerveDrivetrain drivetrain) throws IOException {
    super("output2/output/BarrelRacingV2RB.wpilib.json", false, 10, drivetrain);
  }
}
