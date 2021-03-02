package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class MotionMagicCommand extends CommandBase {

  Translation2d point;
  SwerveDrivetrain drivetrain;

  public MotionMagicCommand(Translation2d point, SwerveDrivetrain drivetrain) {
    this.point = point;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMotionMagic(point.getNorm());
  }

  @Override
  public boolean isFinished() {
    // TODO: Is the translation in meters?
    Translation2d currentPosition = drivetrain.getPose().getTranslation();
    double error = 1; // TODO: Reduce
    return point.getDistance(currentPosition) <= error;
  }

}
