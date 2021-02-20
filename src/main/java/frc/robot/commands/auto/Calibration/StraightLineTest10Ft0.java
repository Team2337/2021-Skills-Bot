package frc.robot.commands.auto.Calibration;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class StraightLineTest10Ft0 extends PathweaverTrajectoryCommand {
    public StraightLineTest10Ft0(SwerveDrivetrain drivetrain) throws IOException {
      super("output/StraightLineTest10Ft0.wpilib.json", drivetrain);
    }
  }