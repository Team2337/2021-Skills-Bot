package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class MotionMagicCommand extends CommandBase {

  Translation2d point;
  SwerveDrivetrain drivetrain;

  /**
   * Move to a point (in feet)
   */
  public MotionMagicCommand(Translation2d point, SwerveDrivetrain drivetrain) {
    this.point = new Translation2d(Units.feetToMeters(point.getX()), Units.feetToMeters(point.getY()));
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.playNote();

    Translation2d currentPosition = drivetrain.getPose().getTranslation();
    Rotation2d heading = new Rotation2d(Math.atan2(point.getY () - currentPosition.getY(), point.getX() - currentPosition.getX()));

    // (10, 0) -> (5, 0) = (5, 0)
    // (10, 0) -> (15, 0) = (15, 0)
    double distance = Units.metersToFeet(point.getNorm());

    drivetrain.setMotionMagic(heading, distance);
  }

  @Override
  public boolean isFinished() {
    Translation2d currentPosition = drivetrain.getPose().getTranslation();
    // Error is in feet - what is the maximum allowable eror distance
    double error = 0.1;

    double distance = currentPosition.getX() - point.getX();
    return Math.abs(distance) <= error;
    // return point.getDistance(currentPosition) <= error;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopNote();

    System.out.println("Ending point: " + point.toString());
  }

}
