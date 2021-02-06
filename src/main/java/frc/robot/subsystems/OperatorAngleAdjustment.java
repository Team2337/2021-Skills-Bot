package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class to change the robot's angle based on an offset.
 * These offsets will be queued on the operator's controller
 * and then put into action on the driver's controller
 * @see OI
 * @author Bryce G., Madison J.
 * @category SWERVE
 */
public class OperatorAngleAdjustment extends SubsystemBase {

  private final Vision vision;

  /* --- Private Double Values --- */
  private double gyroOffset = 0;
  private double farShot;
  private double nearShot;
  private double climbing;
  private double frontTrenchShot;
  private double futureOffsetAngle;
  private double field0;
  private double field90;
  private double field180;
  private double field270;

  private boolean isChangingGyroAngle;
  private boolean limelightRotationMode = false;
  private boolean wasPreviouslyChangingAngle = false;

  private String mode = "";

  private double slowRotateSpeed = 0;

  /* --- Private Boolean Values --- */
  private boolean slowRotateMode = false;
  private boolean ballTrackingEnabled = false;

  /**
   * Class to change the robot's angle based on an offset. These offsets will be
   * queued on the operator's controller and then put into action on the driver's
   * controller
   */
  public OperatorAngleAdjustment(Vision vision) {
    this.vision = vision;

    // Sets all the gyro offsets
    gyroOffset = 0;
    farShot = 9; //25
    nearShot = 0;
    climbing = 28;
    frontTrenchShot = 13;  //???
    field0 = 0;
    field90 = 90;
    field180 = 180;
    field270 = 270;
  }

  /**
   * Sets the future offset angle. Used on operator joystick. The future offset
   * will take affect when the driver button is pressed
   *
   * @param mode - String designating the mode
   * <p>
   * List of modes:
   * </p>
   * <ul>
   * <li><b>farShot</b>
   * - turns the robot to the specified angle, then enables vision,
   * and adjusts the speed of the kicker wheel and shooter to the far
   * shot velocities
   * <li><b>nearShot</b>
   * - turns the robot to the specified angle, then enables vision,
   * and adjusts the speed of the kicker wheel and shooter to the near
   * shot velocities
   * <li><b>climbing</b>
   * - turns the robot towards the hangers, and disables field centric mode
   * <li><b>targetLimelightOn</b>
   * - sets the robot's rotation into vision tracking mode
   * <li><b>0</b>
   * - sets the robot's rotational angle offset to 0 degrees
   * <li><b>90</b>
   * - sets the robot's rotational angle offset to 90 degrees
   * <li><b>180</b>
   * - sets the robot's rotational angle offset to 180 degrees
   * <li><b>270</b>
   * - sets the robot's rotational angle offset to 270 degrees
   * </ul>
   */
  public void setFutureOffsetAngle(String mode) {
    this.mode = mode;
    switch(mode) {
    case "farShot":
      futureOffsetAngle = farShot;
      // Robot.Shooter.setFutureSpeed(Constants.SHOOTSPEEDFAR);
      vision.setRotateLimelight(false);
      // Robot.KickerWheel.setFutureSpeed(Constants.KICKERSPEEDFAR);
      vision.switchPipeLine(1);
      // drivetrain.setFieldOriented(true);
      break;
    case "nearShot":
      futureOffsetAngle = nearShot;
      // Robot.Shooter.setFutureSpeed(Constants.SHOOTSPEEDCLOSE);
      vision.setRotateLimelight(false);
      // Robot.KickerWheel.setFutureSpeed(Constants.KICKERSPEEDCLOSE);
      vision.switchPipeLine(0);
      // drivetrain.setFieldOriented(true);
      break;
    case "climbing":
      futureOffsetAngle = climbing;
      vision.switchPipeLine(2);
      vision.setRotateLimelight(false);
      // drivetrain.setFieldOriented(false);
      break;
    case "targetLimelightOn":
      vision.setRotateLimelight(true);
      // drivetrain.setFieldOriented(true);
      break;
    case "frontTrenchShot":
      futureOffsetAngle = frontTrenchShot;
      vision.switchPipeLine(1);
     // Robot.Shooter.setFutureSpeed(Constants.SHOOTFRONTTRENCHSPEED);
     // Robot.KickerWheel.setFutureSpeed(Constants.KICKERSPEEDFRONTTRENCH);
    case "frontTrenchRunShot":
      futureOffsetAngle = nearShot;
     // Robot.Shooter.setFutureSpeed(Constants.SHOOTSPEEDCLOSE);
      vision.setRotateLimelight(false);
     // Robot.KickerWheel.setFutureSpeed(Constants.KICKERSPEEDCLOSE);
      vision.switchPipeLine(1);
      // drivetrain.setFieldOriented(true);
      break;
    case "0":
      futureOffsetAngle = field0;
      vision.setRotateLimelight(false);
      // drivetrain.setFieldOriented(true);
      break;
    case "90":
      futureOffsetAngle = field90;
      vision.setRotateLimelight(false);
      // drivetrain.setFieldOriented(true);
      break;
    case "180":
      futureOffsetAngle = field180;
      vision.setRotateLimelight(false);
      // drivetrain.setFieldOriented(true);
      break;
    case "270":
      futureOffsetAngle = field270;
      vision.setRotateLimelight(false);
      // drivetrain.setFieldOriented(true);
      break;
    default:
      futureOffsetAngle = 0;
      vision.setRotateLimelight(false);
      // drivetrain.setFieldOriented(true);
    }
  }

  /**
   * Gets the future robot offset angle
   * @return - double value in degrees
   */
  public double getFutureOffsetAngle() {
    return futureOffsetAngle;
  }

  /**
   * Sets the current robot angle offset that the robot will actively attempt to
   * hold
   * @param offsetAngle - double value in degrees
   */
  public void setOffsetAngle(double offsetAngle) {
    this.gyroOffset = offsetAngle;
  }

  /**
   * Gets the current robot angle offset
   * @return - double yaw value in degrees
   */
  public double getGyroAngleOffset() {
    return gyroOffset;
  }

  /**
   * Boolean to track if the offset is currently in use
   * @param isChangingGyroAngle - boolean value (in use: true | holding position: false)
   */
  public void setIsChangingGyroAngle(boolean isChangingGyroAngle) {
    this.isChangingGyroAngle = isChangingGyroAngle;
  }

  /**
   * Gets the state of the boolean value
   * @return - boolean value (in use: true | holding position: false)
   */
  public boolean getIsChangingGyroAngle() {
    return isChangingGyroAngle;
  }

  /**
   * Calculates the robot's angle offset by intaking error, rotation and kP from
   * the swerve drive command to adjust the rotation of the entire robot.
   * @param error - double degree value (current angle - desired angle)
   * @param rotation - double rotation joystick value
   * @param kP - double proportion value used to scale the error to match the rotational units (-1 -> 1)
   * @return - adjusted rotation value acting as a joystick input
   */
  public double calculateGyroOffset(double error, double kP) {
    error %= 360;
    if (error > 180) {
      error -= 360;
    } else if (error < -180) {
      error += 360;
    }

    double rotation = error * kP;
    return (Math.abs(rotation) > 0.6) ? Math.copySign(0.6, rotation) : rotation;
  }

  /**
   * Tells if the limelight mode is queued
   * @param limelightRotationMode - Boolean value (limelightRotationMode: true | limelightRotationMode: false)
   */
  public void setLimelightRotationMode(boolean limelightRotationMode) {
    this.limelightRotationMode = limelightRotationMode;
  }

  /**
   * Gets the limelight mode
   * @return - Boolean value limelight mode
   */
  public boolean getLimelightRotationMode() {
    return limelightRotationMode;
  }

  public void setBallTrackingEnabled(boolean enabled){
    ballTrackingEnabled = enabled;
  }

  /**
   * Lets us know if we are in the ball tracking mode, we are rotating using the pixycam
   * @return whether or not we are tracking balls
   */
  public boolean getBallTrackingEnabled() {
    return ballTrackingEnabled;
  }

  /**
   * Gets the mode for the future offset angle
   * @return - String value currently set future mode
   */
  public String getMode() {
    return mode;
  }

  /**
   * Sets the slow rotate mode on the robot to on or off
   * @param slowRotateMode - boolean value indicating the slow rotate mode (slow rotate on: true | slow rotate off: false)
   */
  public void setSlowRotateMode(boolean slowRotateMode) {
    this.slowRotateMode = slowRotateMode;
  }

  /**
   * Gets the slow rotate mode boolean indicating whether slow rotate is occurring
   * or not
   * @return - boolean value indicating the slow rotate mode (slow rotate on: true | slow rotate off: false)
   */
  public boolean getSlowRotateMode() {
    return slowRotateMode;
  }

  /**
   * Sets the speed of the slow rotation
   * @param speed - double value for the slow rotation speed (-1 -> 1)
   */
  public void setSlowRotateSpeed(double speed) {
    this.slowRotateSpeed = speed;
  }

  /**
   * Gets the speed for slow rotation
   * @return - double speed value for slow rotation (-1 -> 1)
   */
  public double getSlowRotateSpeed() {
    return this.slowRotateSpeed;
  }

  /**
   * Sets the mode if the robot was previously rotating.
   * (was rotating: true | wasn't rotating: false)
   * @param wasPreviouslyChangingAngle - boolean value
   */
  public void setPreviouslyChangingGyroAngle(boolean wasPreviouslyChangingAngle) {
    this.wasPreviouslyChangingAngle = wasPreviouslyChangingAngle;
  }

  /**
   * Gets the previous rotation mode (was rotating: true | wasn't rotating: false)
   * @return - boolean value
   */
  public boolean wasChangingGyroAngle() {
    return this.wasPreviouslyChangingAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("SlowRotate", getSlowRotateMode());
  }
}