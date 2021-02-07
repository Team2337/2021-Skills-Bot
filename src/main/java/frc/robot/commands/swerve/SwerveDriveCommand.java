package frc.robot.commands.swerve;

import frc.robot.Utilities;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.*;

/**
 * Command running the swerve calculations with the joystick
 *
 * @see SwerveDrivetrain
 * @author Bryce G.
 * @category SWERVE
 */
public class SwerveDriveCommand extends CommandBase {

  private boolean swerveDebug = false;

  private final SwerveDrivetrain drivetrain;
  private final XboxController controller;

  /**
   * Command running the swerve calculations with the joystick
   *
   * @param subsystem - SwerveDrivetrain subsystem object
   */
  public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.controller = controller;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    /* --- Joystick Values --- */
    double forward = -controller.getY(Hand.kLeft);
    double strafe = controller.getX(Hand.kLeft);
    double rotation = -controller.getX(Hand.kRight);

    // Set Deadband
    forward = Utilities.deadband(forward, 0.1);
    strafe = Utilities.deadband(strafe, 0.1);
    rotation = Utilities.deadband(rotation, 0.1);

    // Pass on joystick values to be calculated into angles and speeds
    drivetrain.calculateJoystickInput(forward, strafe, rotation);

    if(swerveDebug) {
      SmartDashboard.putNumber("Forward", forward);
      SmartDashboard.putNumber("Strafe", strafe);
      SmartDashboard.putNumber("Rotation", rotation);
    }
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
