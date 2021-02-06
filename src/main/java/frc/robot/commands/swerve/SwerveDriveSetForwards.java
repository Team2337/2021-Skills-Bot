package frc.robot.commands.swerve;

import frc.robot.Utilities;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Sets the forwards value to a set a mock joystick value
 * @see SwerveDrivetrain
 * @author Bryce G.
 * @category SWERVE
 */
public class SwerveDriveSetForwards extends CommandBase {

  private final SwerveDrivetrain swerveDrivetrain;

  private double forward = 0;

  /**
   * Sets the forwards value to a set a mock joystick value
   * @param subsystem - SwerveDrivetrain Subsystem object
   * @param forward - mock forward joystick value
   */
  public SwerveDriveSetForwards(SwerveDrivetrain subsystem, double forward) {
    this.swerveDrivetrain = subsystem;
    addRequirements(subsystem);

    this.forward = forward;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // Square Joystick Inputs
    forward = Utilities.squareValues(forward);

    // Set Deadband
    forward = Utilities.deadband(forward, 0.1);

    // Smartdashboard prints
    SmartDashboard.putNumber("Forward", forward);

    // Pass on joystick values to be calculated into angles and speeds
    swerveDrivetrain.calculateJoystickInput(forward, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    // In the event this command stops, we don't want the motors to move
    swerveDrivetrain.stopAngleMotors();
    swerveDrivetrain.stopDriveMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
