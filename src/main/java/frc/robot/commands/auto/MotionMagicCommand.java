package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class MotionMagicCommand extends CommandBase {

  Translation2d point;
  Rotation2d heading;
  SwerveDrivetrain drivetrain;

  /**
   * Move to a point (in feet)
   */
  public MotionMagicCommand(Translation2d point, SwerveDrivetrain drivetrain) {
    Translation2d currentPosition = drivetrain.getPose().getTranslation();
    this.heading = new Rotation2d(Math.atan2(point.getY() - currentPosition.getY(), point.getX() - currentPosition.getX()));
    this.point = new Translation2d(Units.feetToMeters(point.getX()), Units.feetToMeters(point.getY()));
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMotionMagic(Units.metersToFeet(point.getNorm()));
    drivetrain.setModuleAngle(heading);
  }

  @Override
  public boolean isFinished() {
    Translation2d currentPosition = drivetrain.getPose().getTranslation();
    double error = 1; // TODO: Reduce
    return point.getDistance(currentPosition) <= error;
  }

}
