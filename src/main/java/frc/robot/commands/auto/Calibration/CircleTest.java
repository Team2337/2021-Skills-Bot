package frc.robot.commands.auto.calibration;

import java.io.IOException;

import frc.robot.commands.auto.PathweaverTrajectoryCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class CircleTest extends PathweaverTrajectoryCommand {
  public CircleTest(SwerveDrivetrain drivetrain) throws IOException {
    super("output/circleTest.wpilib.json", drivetrain);
  }
}