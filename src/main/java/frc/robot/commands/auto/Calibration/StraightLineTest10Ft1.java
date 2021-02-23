package frc.robot.commands.auto.calibration;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class StraightLineTest10Ft1 extends PathweaverTrajectoryCommand {
    public StraightLineTest10Ft1(SwerveDrivetrain drivetrain) throws IOException {
      super("output/StraightLineTest10Ft1.wpilib.json", drivetrain);
    }
  }