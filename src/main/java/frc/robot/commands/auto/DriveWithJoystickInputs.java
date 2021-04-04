package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

public class DriveWithJoystickInputs extends CommandBase {

    private SwerveDrivetrain drivetrain;
    private Pigeon pigeon;
    private double forward;
    private double strafe = 0;
    private double rotation = 20;
    private double encoderDist = 0;
    private double endAngleDegree;
    private double currentGyro;
    private double rotationError;
    private double endDistance;
    private int monitorModule;

    private double rotationP = 0.009; 
    private double maxRotationSpeed = 0.15;
    

    public DriveWithJoystickInputs(SwerveDrivetrain drivetrain, double encoderDist, double forward, double strafe, double endAngleDegree, int monitorModule, Pigeon pigeon) { //, double forwardDist, double horizontalDist, double endAngleDegree) {
        this.drivetrain = drivetrain;
        this.encoderDist = encoderDist;
        this.strafe = strafe;
        this.forward = forward;
        this.endAngleDegree = endAngleDegree;
        this.pigeon = pigeon;
        this.monitorModule = monitorModule;
        addRequirements(drivetrain);
      } 

      public DriveWithJoystickInputs(SwerveDrivetrain drivetrain, double encoderDist, double forward, double strafe, double rotation, int monitorModule) { //, double forwardDist, double horizontalDist, double endAngleDegree) {
        this.drivetrain = drivetrain;
        this.encoderDist = encoderDist;
        this.strafe = strafe;
        this.forward = forward;
        this.rotation = rotation;
        this.monitorModule = monitorModule;
        addRequirements(drivetrain);
      } 

      @Override
      public void initialize() {
        endDistance =  drivetrain.getTicksInches(encoderDist);
      }

      @Override
      public void execute() {
        if (rotation == 20) {
        currentGyro = pigeon.getYawMod();
        rotationError = (endAngleDegree - currentGyro);
        rotation = rotationError * rotationP;
        rotation = rotation > maxRotationSpeed ? maxRotationSpeed : rotation;
      }
       
       // Pass on joystick values to be calculated into angles and speeds
       drivetrain.calculateJoystickInput(forward, strafe, rotation, false);
       //drivetrain.setAngleMotorsTeleop(moduleAngle);
      }

      @Override
      public void end(boolean interrupted) {
        System.out.println("Encoder Ticks: " + drivetrain.getDriveMotorPosition(monitorModule));
      }

      @Override
      public boolean isFinished() {
        return (Math.abs(drivetrain.getDriveMotorPosition(monitorModule)) > endDistance);
      }



}