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

  private final SwerveDrivetrain drivetrain;
  private final XboxController driverController;
  private final XboxController operatorController;

  /**
   * Command running the swerve calculations with the joystick
   *
   * @param subsystem - SwerveDrivetrain subsystem object
   */
  public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController driverController, XboxController operatorController) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.driverController = driverController;
    this.operatorController = operatorController;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // Inverting this Y value because Xbox controllers return negative values when we push forward.
    double forward = -driverController.getY(Hand.kLeft);


    // Inverting X values because we want positive values when we pull to the left.
    // Xbox controllers return positive values when you pull to the right by default.
    double strafe = -driverController.getX(Hand.kLeft); 
    double rotation = -driverController.getX(Hand.kRight) * 0.34;
    // Inverting the bumper value because we want field-oriented drive by default.
    boolean isFieldOriented = !driverController.getBumper(Hand.kLeft);
    double speedLimit = .75;
    //if (operatorController.getTriggerAxis(Hand.kRight) > .5) {
      forward = forward * speedLimit;
      strafe = strafe * speedLimit;
   // }

    //forward = Math.copySign(forward * forward, forward);
    //strafe = Math.copySign(strafe * strafe, strafe);

    forward = Utilities.deadband(forward, 0.04);
    strafe = Utilities.deadband(strafe, 0.04);
    rotation = Utilities.deadband(rotation, 0.04);

    // Pass on joystick values to be calculated into angles and speeds
    drivetrain.calculateJoystickInput(forward, strafe, rotation, isFieldOriented);

    SmartDashboard.putNumber("Forward", forward);
    SmartDashboard.putNumber("Strafe", strafe);
    SmartDashboard.putNumber("Rotation", rotation);
    SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);
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
