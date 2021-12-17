package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class SDTargetAndDistance extends CommandBase {
  
  private final XboxController controller;
  private final SwerveDrivetrain drivetrain;
  private final Vision vision;

  private double futureAngle = 45;
  private double futureAngleActive = 0;


  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /**
   * Command running the swerve calculations with the joystick
   *
   * @param subsystem - SwerveDrivetrain subsystem object
   */

  public SDTargetAndDistance(XboxController controller, SwerveDrivetrain drivetrain, Vision vision) {
    this.controller = controller;
    this.drivetrain = drivetrain;
    this.vision = vision;
    //this.pigeon = pigeon;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double vxMetersPerSecond = -Utilities.modifyAxis(controller.getY(Hand.kLeft)) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    double vyMetersPerSecond = -Utilities.modifyAxis(controller.getX(Hand.kLeft)) * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    double omegaRadiansPerSecond = -Utilities.modifyAxis(controller.getX(Hand.kRight)) * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    boolean isFieldOriented = !controller.getBumper(Hand.kLeft);
    boolean isVisionTargeting = !controller.getRawButton(1);

    if(controller.getBumper(Hand.kRight)) {
      futureAngleActive = futureAngle;
    } else {
      futureAngleActive = 0;
    }


   // Calculate range
   double range =
    Utilities.calculateDistanceToTargetMeters(
       CAMERA_HEIGHT_METERS,
       TARGET_HEIGHT_METERS,
       CAMERA_PITCH_RADIANS,
       Units.degreesToRadians(vision.getLimelightPitchDegrees())
    );

   if(vision.ifLimelightHasTarget() && isVisionTargeting){
    System.out.println("We made it!!!!!!!!");
    drivetrain.drive(
     ChassisSpeeds.fromFieldRelativeSpeeds(
       vxMetersPerSecond,
       -forwardController.calculate(range, GOAL_RANGE_METERS),
       -turnController.calculate(vision.getLimelightYawDegrees(), 0),
       drivetrain.getGyroscopeRotation()
      )
    );
   } else if (isFieldOriented) {
    drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        vxMetersPerSecond,
        vyMetersPerSecond,
        omegaRadiansPerSecond,
        drivetrain.getGyroscopeRotation(futureAngleActive)

      )
    ); 
   } else {
    drivetrain.drive(new ChassisSpeeds(
      vxMetersPerSecond,
      vyMetersPerSecond,
      omegaRadiansPerSecond
     )
    );
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
