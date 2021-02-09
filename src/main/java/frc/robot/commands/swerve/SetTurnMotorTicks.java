package frc.robot.commands.swerve;

import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Set swerve drive modules to a specific position based on SmartDashboard entry for "ticks"
 *
 * @see SwerveDrivetrain
 * @author 2337.
 * @category SWERVE
 */
public class SetTurnMotorTicks extends CommandBase {

  private final SwerveDrivetrain drivetrain;

  public SetTurnMotorTicks(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
   drivetrain.setTurnMotorTicks();
  }

  @Override
  public void end(boolean interrupted) {
    // In the event this command stops, we don't want the motors to move
    drivetrain.stopAngleMotors();
    drivetrain.stopDriveMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
