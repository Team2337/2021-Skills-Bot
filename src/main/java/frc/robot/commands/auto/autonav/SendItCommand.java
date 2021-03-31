package frc.robot.commands.auto.autonav;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

public class SendItCommand extends CommandBase {

  private SwerveDrivetrain drivetrain;
  private Pigeon pigeon;

  /**
   * woke up from a nap, its a little dark out
   * but what are you silly? i'm still gonna send it
   */
  public SendItCommand(SwerveDrivetrain drivetrain, Pigeon pigeon) {
    this.drivetrain = drivetrain;
    this.pigeon = pigeon;
  }

  @Override
  public void execute() {
    double kFullSpeed = Units.feetToMeters(20);
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      kFullSpeed * -1, // Negated, since we need to move towards our alliance wall in field relative terms
      Units.feetToMeters(0),
      0,
      Rotation2d.fromDegrees(pigeon.getYaw())
    );
    SwerveModuleState[] moduleStates = drivetrain.getKinematics().toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, kFullSpeed);
    drivetrain.setModuleStates(moduleStates);
  }

}