package frc.robot.commands.auto.autonav;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

public class SendItWithJoystick extends CommandBase {

  private SwerveDrivetrain drivetrain;
  private Pigeon pigeon;

  /**
   * woke up from a nap, its a little dark out but what are you silly i'm still
   * gonna send it!
   */
  public SendItWithJoystick(SwerveDrivetrain drivetrain, Pigeon pigeon) {
    this.drivetrain = drivetrain;
    this.pigeon = pigeon;
  }

  @Override
  public void initialize() {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      Units.feetToMeters(-13.6), // x - might need to be negative?
      Units.feetToMeters(0), // y
      0,
      Rotation2d.fromDegrees(pigeon.getYaw())
    );
    SwerveModuleState[] moduleStates = drivetrain.getKinematics().toSwerveModuleStates(chassisSpeeds);
    drivetrain.setModuleStates(moduleStates);
  }

  // TODO: This is going to need an end, or we're going to just keep going

}