package frc.robot.commands.auto;

import java.util.List;
import frc.robot.subsystems.PixyCam;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.GalacticSearchAuton;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Uses the PixyCam to run an auton based on the position of a ball.
 * @author Michael F., Zach O., and Nicholas S.
 */
public final class GalacticSearch extends InstantCommand {

  /**
   * Creates the Galactic Search Command.
   *
   * @param pixy The PixyCam used by this command.
   */
  public GalacticSearch(PixyCam pixy, SwerveDrivetrain drivetrain) {
    super(() -> {
      /**
       * The x-offset (in inches) of the robot from the start of the starting
       * zone to position it perfectly with its bumpers touching the edge of
       * the starting zone.
       * This needs to be declared in the super or Java will get mad at me.
       */
      final double X_OFFSET = 30 - (Constants.DRIVETRAIN_LENGTH_INCHES / 2);

      //Get PixyCam values
      int x = pixy.getLargestTargetX();

      //Create ball positions list
      List<Pose2d> ballPositions = List.of();

      //If we don't use the trajectory generator, use the enum:
      GalacticSearchAuton auton;

      //Source of positions: https://firstfrc.blob.core.windows.net/frc2021/Manual/2021AtHomeChallengesManual.pdf
      //The values used with x represent the x-position of the closest target in the field of view of the camera.
      if (x >= 0 && x < 79) {
        auton = GalacticSearchAuton.RedA;
        //Red A path
        ballPositions = List.of(
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(90 - X_OFFSET),
              Units.inchesToMeters(90)),
            Rotation2d.fromDegrees(0)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(150 - X_OFFSET),
              Units.inchesToMeters(60)),
            Rotation2d.fromDegrees(26.57)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(180 - X_OFFSET),
              Units.inchesToMeters(150)),
            Rotation2d.fromDegrees(-71.57))
        );
      } else if(x >= 79 && x < 158) {
        auton = GalacticSearchAuton.RedB;
        //Red B path
        ballPositions = List.of(
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(90 - X_OFFSET),
              Units.inchesToMeters(120)),
            Rotation2d.fromDegrees(-23.65)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(150 - X_OFFSET),
              Units.inchesToMeters(60)),
            Rotation2d.fromDegrees(45)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(210 - X_OFFSET),
              Units.inchesToMeters(120)),
            Rotation2d.fromDegrees(-45))
        );
      } else if(x >= 158 && x < 237) {
        auton = GalacticSearchAuton.BlueA;
        //Blue A path
        ballPositions = List.of(
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(180 - X_OFFSET),
              Units.inchesToMeters(30)),
            //We once again need to take into account the offset starting position for this angle.
            Rotation2d.fromDegrees(-20.73)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(210 - X_OFFSET),
              Units.inchesToMeters(120)),
              Rotation2d.fromDegrees(71.57)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(270 - X_OFFSET),
              Units.inchesToMeters(90)),
              Rotation2d.fromDegrees(-26.57))
        );
      } else if(x >= 237 && x <= 315) {
        auton = GalacticSearchAuton.BlueB;
        //Blue B path
        ballPositions = List.of(
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(180 - X_OFFSET),
              Units.inchesToMeters(60)),
            Rotation2d.fromDegrees(-10.72)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(240 - X_OFFSET),
              Units.inchesToMeters(120)),
            Rotation2d.fromDegrees(45)),
          new Pose2d(
            new Translation2d(
              Units.inchesToMeters(300 - X_OFFSET),
              Units.inchesToMeters(60)),
            Rotation2d.fromDegrees(-45))
        );
      }

      //If we didn't see a ball, do nothing (return)
      if (ballPositions.size() < 1) {
        return;
      }

      //Add end position
      //End position is a full robot's length in to the end zone + whatever the Y value of the last ball is
      ballPositions.add(
        new Pose2d(
          Units.inchesToMeters(330 + Constants.DRIVETRAIN_LENGTH_INCHES - X_OFFSET),
          ballPositions.get(ballPositions.size() - 1).getY(),
          new Rotation2d(0)));

      CommandScheduler.getInstance().schedule(
        new TrajectoryCommand(
          //Generate a trajectory
          TrajectoryGenerator.generateTrajectory(ballPositions, Constants.SWERVE_TRAJECTORY_CONFIG),
          drivetrain
        )
      );
    }, pixy);
  }

}