package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

public class RotateToDegree extends PIDCommand{
    private final SwerveDrivetrain drivetrain;
    public RotateToDegree(double degree, SwerveDrivetrain drivetrain, Pigeon pigeon) {
        super(
            new PIDController(.005,0,0),
            pigeon::getYaw,
            degree,
            output -> drivetrain.calculateJoystickInput(0, 0, output, false),
            drivetrain
        );
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(2);

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopAngleMotors();
        drivetrain.stopDriveMotors();
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
    
}
