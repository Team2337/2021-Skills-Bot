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
public class GalacticSearch extends InstantCommand {

  /**
   * Creates the Galactic Search Command.
   *
   * @param pixy The PixyCam used by this command.
   */
  public GalacticSearch(PixyCam pixy, SwerveDrivetrain drivetrain) {
    super(() -> {
      pixy.updateTargets();
      int x = pixy.getLargestTargetX();

      List<Pose2d> ballPositions = List.of();
      TrajectoryConfig config = new TrajectoryConfig(
        Units.feetToMeters(Constants.Swerve.MAX_FEET_PER_SECOND),
        Units.feetToMeters(Constants.Swerve.MAX_FEET_PER_SECOND)
      );

      //Source of positions: https://firstfrc.blob.core.windows.net/frc2021/Manual/2021AtHomeChallengesManual.pdf
      //The values used with x represent the x-position of the closest target in the field of view of the camera.
      if (x >= 0 && x < 79) {
        //Red A path
        ballPositions = List.of(
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(90 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(90)),
            //The robot does not need to turn for this first ball
            new Rotation2d(0)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(150 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(60)),
            //The numbers used in these angles are feet so they remain relatively small.
            //Angle between last coords and these coords
            new Rotation2d(Math.atan2(2.5, 5))),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(180 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(150)),
            //Angle between last coords and these coords
            new Rotation2d(Math.atan2(-7.5, 2.5)))
        );
      } else if(x >= 79 && x < 158) {
        //Red B path
        ballPositions = List.of(
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(90 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(120)),
            //We need to take into account the offset starting position for this angle.
            //Also, these tangents are measuring feet. The offset is stored in inches.
            new Rotation2d(Math.atan2(-2.5, 7.5 - (Constants.GSC_X_OFFSET / 12)))),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(150 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(60)),
            //Actually solving this turns into a 45 degree angle,
            //so we convert it to radians.
            new Rotation2d(Math.toRadians(45))),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(210 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(120)),
            //See previous rotation comment
            new Rotation2d(Math.toRadians(-45)))
        );
      } else if(x >= 158 && x < 237) {
        //Blue A path
        ballPositions = List.of(
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(180 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(30)),
            //We once again need to take into account the offset starting position for this angle.
            new Rotation2d(Math.atan2(-5, 15 - Constants.GSC_X_OFFSET))),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(210 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(120)),
            new Rotation2d(7.5, 2.5)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(270 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(90)),
            new Rotation2d(-2.5, 5))
        );
      } else if(x >= 237 && x <= 315) {
        //Blue B path
        ballPositions = List.of(
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(180 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(60)),
            //We need to take into account the offset again
            new Rotation2d(2.5, 15 - Constants.GSC_X_OFFSET)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(240 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(120)),
            //Once again, solving this turns into a 45 degree angle,
            //so we convert it to radians.
            new Rotation2d(Math.toRadians(45))),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(300 - Constants.GSC_X_OFFSET),
              Units.inchesToMeters(60)),
            //See previous rotation comment
            new Rotation2d(Math.toRadians(-45)))
        );
      }

      //If we didn't see a ball, do nothing (return)
      if (ballPositions.size() < 1) {
        return;
      }

      //Add end position
      //End position is a full robot's length in to the end zone + whatever the Y value of the last ball is
      ballPositions.add(new Pose2d(Units.inchesToMeters(330 + Constants.DRIVETRAIN_LENGTH_INCHES - Constants.GSC_X_OFFSET), ballPositions.get(ballPositions.size() - 1).getY(), new Rotation2d(0)));

      CommandScheduler.getInstance().schedule(
        new TrajectoryCommand(
          //Old code in case we need it again
          /* TrajectoryGenerator.generateTrajectory(
            // Start at 30x90 - move the robot back by half to make it so the front of the robot is flat to the starting plane
            new Pose2d(Units.inchesToMeters(30 - (Constants.DRIVETRAIN_LENGTH_INCHES / 2)), Units.inchesToMeters(90), new Rotation2d(0)),
            // Move to the ball positions depending on the start input
            ballPositions,
            // End position is a full robot's length in to the end zone + whatever the Y value of the last ball is
            new Pose2d(Units.inchesToMeters(330 + Constants.DRIVETRAIN_LENGTH_INCHES), ballPositions.get(ballPositions.size() - 1).getY(), new Rotation2d(0)),
            config
          ), drivetrain) */

          //Generate a trajectory
          TrajectoryGenerator.generateTrajectory(ballPositions, config),
          drivetrain
        )
      );
    }, pixy);
  }

}