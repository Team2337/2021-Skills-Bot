package frc.robot.commands.auto;

import java.util.List;
import frc.robot.subsystems.PixyCam;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Uses the PixyCam to run an auton based on the position of a ball.
 * @author Michael F., Zach O., and Nicholas S.
 */
public class GalacticSearchCommand extends InstantCommand {

  /**
   * Creates the Galactic Search Command.
   *
   * @param pixy The PixyCam used by this command.
   */
  public GalacticSearchCommand(PixyCam pixy, SwerveDrivetrain drivetrain) {
    super(() -> {
      pixy.updateTargets();
      int x = pixy.getLargestTargetX();

      List<Translation2d> ballPositions = List.of();
      TrajectoryConfig config = new TrajectoryConfig(
        Units.feetToMeters(Constants.Swerve.MAX_FEET_PER_SECOND),
        Units.feetToMeters(Constants.Swerve.MAX_FEET_PER_SECOND)
      );

      if (x >= 0 && x < 79) {
        //Red A path
        ballPositions = List.of(
          new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(90)),
          new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)),
          new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(150))
        );
      } else if(x >= 79 && x < 158) {
        //Red B path
        ballPositions = List.of(
          new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(120)),
          new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)),
          new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120))
        );
      } else if(x >= 158 && x < 237) {
        //Blue A path
        ballPositions = List.of(
          new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(30)),
          new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120)),
          new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(90))
        );
      } else if(x >= 237 && x <= 315) {
        //Blue B path
        ballPositions = List.of(
          new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(60)),
          new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(120)),
          new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(60))
        );
      }

      //If we didn't see a ball, do nothing (return)
      if (ballPositions.size() < 1) {
        return;
      }

      CommandScheduler.getInstance().schedule(
        new TrajectoryCommand(
          TrajectoryGenerator.generateTrajectory(
            // Start at 30x90 - move the robot back by half to make it so the front of the robot is flat to the starting plane
            new Pose2d(Units.inchesToMeters(30 - (Constants.DRIVETRAIN_LENGTH_INCHES / 2)), Units.inchesToMeters(90), new Rotation2d(0)),
            // Move to the ball positions depending on the start input
            ballPositions,
            // End position is a full robot's length in to the end zone + whatever the Y value of the last ball is
            new Pose2d(Units.inchesToMeters(330 + Constants.DRIVETRAIN_LENGTH_INCHES), ballPositions.get(ballPositions.size() - 1).getY(), new Rotation2d(0)),
            config
          ), drivetrain)
      );
    }, pixy);
  }

}