package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.subsystems.OperatorAngleAdjustment;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Sets the forwards value to a set a mock joystick value
 * @see SwerveDrivetrain
 * @author Bryce G.
 * @category SWERVE
 */
public class AutoDriveWithJoystickInput extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final OperatorAngleAdjustment operatorAngleAdjustment;
  private final Pigeon pigeon;

  private double forward;
  private double strafe;
  private double rotation;

  private double forwardDist;

  private double endAngleDegree;
  private double currentGyro;
  private double rotationError;

  private double rotationP = 0.009;
  private double maxRotationSpeed = 0.15;
  private double encoderDist = 0;

  /**
   * Sets the forwards value to a set a mock joystick value
   *
   * @param drivetrain - drivetrain Subsystem object
   * @param pid
   * @param forward    - mock forward joystick value
   */
  public AutoDriveWithJoystickInput(SwerveDrivetrain drivetrain, OperatorAngleAdjustment operatorAngleAdjustment, Pigeon pigeon, double encoderDist, double forwardDist, double horizontalDist, double endAngleDegree) {
    this.drivetrain = drivetrain;
    this.operatorAngleAdjustment = operatorAngleAdjustment;
    this.pigeon = pigeon;
    addRequirements(drivetrain, operatorAngleAdjustment, pigeon);

    this.encoderDist = encoderDist;
    this.forwardDist = forwardDist;
    this.strafe = horizontalDist * Constants.Auton.INCHESTOJOYSTICKVALUE;
    this.forward = forwardDist * Constants.Auton.INCHESTOJOYSTICKVALUE;
    this.endAngleDegree = endAngleDegree;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    currentGyro = pigeon.getYawMod();
    rotationError = (endAngleDegree - currentGyro);
    rotation = rotationError * rotationP;
    rotation = rotation > maxRotationSpeed ? maxRotationSpeed : rotation;

   // Pass on joystick values to be calculated into angles and speeds
   drivetrain.calculateJoystickInput(forward, strafe, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    operatorAngleAdjustment.setOffsetAngle(pigeon.getYawMod());
    System.out.println("Encoder Ticks: " + drivetrain.getModule(3).getDriveEncoderValue());
    drivetrain.zeroAllDriveEncoders();
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(drivetrain.getModule(3).getDriveEncoderValue()) > encoderDist * Constants.Swerve.TICKSPERINCH);
  }
}