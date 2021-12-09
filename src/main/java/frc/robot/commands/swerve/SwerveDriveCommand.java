package frc.robot.commands.swerve;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier fieldOrientedSupplier;

  /**
   * Command running the swerve calculations with the joystick
   *
   * @param subsystem - SwerveDrivetrain subsystem object
   */
  public SwerveDriveCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, BooleanSupplier fieldOrientedSupplier, SwerveDrivetrain drivetrain) {
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldOrientedSupplier = fieldOrientedSupplier;

    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    if (fieldOrientedSupplier.getAsBoolean()) {
      drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          translationXSupplier.getAsDouble(),
          translationYSupplier.getAsDouble(),
          rotationSupplier.getAsDouble(),
          drivetrain.getGyroscopeRotation()
        )
      );
    } else {
      drivetrain.drive(new ChassisSpeeds(
        translationXSupplier.getAsDouble(),
        translationYSupplier.getAsDouble(),
        rotationSupplier.getAsDouble()
      ));
    }
  }

  @Override
  public void end(boolean interrupted) {
    // In the event this command stops, we don't want the motors to move
    drivetrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
