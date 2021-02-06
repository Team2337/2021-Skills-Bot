package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.subsystems.OperatorAngleAdjustment;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private final OperatorAngleAdjustment operatorAngleAdjustment;
  private final Pigeon pigeon;
  private final Vision vision;

  private final OI oi;
  /**
   * Value from the Y axis on the left joystick, on the driver controller
   */
  private double forward = 0;

  /**
   * Value from the X axis on the left joystick, on the driver controller
   */
  private double strafe = 0;

  /**
   * Value from the X axis on the right joystick, on the driver controller
   */
  private double rotation = 0;
  private double error;
  private double kP;

  /** Rotation value of the previous iteration */
  private double lastRotation;
  /** Deadband for the rotational input  */
  private double rotationDeadband = 0.1;
  /** Rotational P while not rotating */
  private double stationaryP = 0.015;
  /** Rotational P while rotating */
  private double movingP = 0.01; //0.007

  /**
   * Command running the swerve calculations with the joystick
   *
   * @param subsystem - SwerveDrivetrain subsystem object
   */
  public SwerveDriveCommand(SwerveDrivetrain drivetrain, OperatorAngleAdjustment operatorAngleAdjustment, Pigeon pigeon, Vision vision, OI oi) {
    this.drivetrain = drivetrain;
    this.operatorAngleAdjustment = operatorAngleAdjustment;
    this.pigeon = pigeon;
    this.vision = vision;
    addRequirements(drivetrain, operatorAngleAdjustment, pigeon, vision);

    this.oi = oi;

    if(Robot.isComp) {
      stationaryP = 0.007;
      movingP = 0.002;
    } else {
      stationaryP = 0.015;
      movingP = 0.007;
    }
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    /* --- Joystick Values --- */
    forward = -oi.driverJoystick.getRawAxis(1);
    strafe = oi.driverJoystick.getRawAxis(0);
    rotation = -oi.driverJoystick.getRawAxis(4);

    // Set Deadband
    forward = Utilities.deadband(forward, 0.1);
    strafe = Utilities.deadband(strafe, 0.1);
    rotation = Utilities.deadband(rotation, 0.1);

    /* --- Rotation Angle Adjustments --- */
    if (Math.abs(rotation) > rotationDeadband) {
      lastRotation = rotation;
    } else {
      // Checks to see if we were rotating in the previous iteration, but are not currently rotating
      if (Math.abs(lastRotation) > rotationDeadband && Math.abs(rotation) <= rotationDeadband) {
        operatorAngleAdjustment.setOffsetAngle(pigeon.getYawMod());
        rotation = 0;
        lastRotation = rotation;
      }
      // Checks to see if the Driver's button is being pressed, and sets the current offset angle
      if (operatorAngleAdjustment.getIsChangingGyroAngle()) {
        operatorAngleAdjustment.setOffsetAngle(operatorAngleAdjustment.getFutureOffsetAngle());
      }
      // Sets the error of the robot's angle offset & current gyro angle
      error = operatorAngleAdjustment.getGyroAngleOffset() - pigeon.getYawMod();
      kP = forward == 0 && strafe == 0 ? stationaryP : movingP;
      if(error > 180) {
        error -= 360;
      } else if(error < -180) {
        error += 360;
      }
      rotation = operatorAngleAdjustment.calculateGyroOffset(error, kP);
    }

    // Checks the limelight mode to rotate towards the target
    if(operatorAngleAdjustment.getLimelightRotationMode()) {
      double tx = 0;
      if(vision.getPipeline() == 0 /**&& Robot.Shooter.getAvgRPM() > 250*/) {
        tx = -(Math.toRadians(vision.getDoubleValue("tx") - 2));
      } else if (vision.getPipeline() == 1) {
        tx = -(Math.toRadians(vision.getDoubleValue("tx")));
      }
      if(vision.getPipeline() == 1) {
        if(Math.abs(tx) <  Math.toRadians(2)) {
          rotation = (tx * Constants.VISIONCLOSEROTATIONP);
        } else if(Math.abs(tx) < Math.toRadians(5)) {
          rotation = (tx * Constants.VISIONMIDDLEROTATIONP);
        } else {
          rotation = (tx * Constants.VISIONOFFROTATIONP);
        }
      } else {
        if(Math.abs(tx) <  Math.toRadians(2)) {
          rotation = (tx * Constants.VISIONCLOSEROTATIONP);
        } else if(Math.abs(tx) < Math.toRadians(5)) {
          rotation = (tx * Constants.VISIONMIDDLEROTATIONP);
        } else {
          rotation = (tx * Constants.VISIONOFFROTATIONP);
        }
      }
    }

    if(operatorAngleAdjustment.getBallTrackingEnabled()){
      if(vision.pixyRightDigital.get()) {
        rotation = -(Math.toRadians(vision.getPixyRightValue() - 2) * Constants.BALLTRACKINGP);
      } else {
        rotation = 0;
      }
    }

    // Checks to see if we are in slow rotate mode, then directly sets the rotation to the given speed
    if(operatorAngleAdjustment.getSlowRotateMode()) {
      rotation = operatorAngleAdjustment.getSlowRotateSpeed();
      if(Math.abs(error) >= 25) {
        operatorAngleAdjustment.setSlowRotateMode(false);
        operatorAngleAdjustment.setOffsetAngle(pigeon.getYawMod());
      }
    }

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
