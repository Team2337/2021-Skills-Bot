package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

public class DriveWithJoystickInputs extends CommandBase {

    private SwerveDrivetrain drivetrain;
    private double forward;
    private double strafe = 0;
    private double rotation = 0;
    private double encoderDist = 0;
    private double endDistance;
    private double moduleAngle;

    public DriveWithJoystickInputs(SwerveDrivetrain drivetrain, double forward, double rotation, double encoderDist, double moduleAngle) { //, double forwardDist, double horizontalDist, double endAngleDegree) {
        this.drivetrain = drivetrain;
        this.encoderDist = encoderDist;
        this.moduleAngle = moduleAngle;
        this.forward = forward;
        this.rotation = rotation;
        addRequirements(drivetrain);
      } 

      @Override
      public void initialize() {
       endDistance =  drivetrain.getTicksInches(encoderDist);
      }

      @Override
      public void execute() {
          /*
        currentGyro = Pigeon.getPigeonYawMod();
        rotationError = (endAngleDegree - currentGyro);
        rotation = rotationError * rotationP;
        rotation = rotation > maxRotationSpeed ? maxRotationSpeed : rotation;
       */
       // Pass on joystick values to be calculated into angles and speeds
       drivetrain.calculateJoystickInput(forward, strafe, rotation, false);
       //drivetrain.setAngleMotorsTeleop(moduleAngle);
      }

      @Override
      public void end(boolean interrupted) {
        System.out.println("Encoder Ticks: " + drivetrain.getDriveMotorPosition(0));
      }

      @Override
      public boolean isFinished() {
        return (Math.abs(drivetrain.getDriveMotorPosition(0)) > endDistance);
      }



}