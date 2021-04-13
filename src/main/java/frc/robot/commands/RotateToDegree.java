package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Utilities;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

public class RotateToDegree extends PIDCommand{
    private final SwerveDrivetrain drivetrain;
    public RotateToDegree(double degree, double kP, SwerveDrivetrain drivetrain, Pigeon pigeon) {
        super(
            new PIDController(Utilities.scaleToRange(degree, -180, 180, -1, 1) * kP, 0, 0),
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
    public void execute() {
      m_useOutput.accept(
          m_controller.calculate(m_measurement.getAsDouble(), m_setpoint.getAsDouble()));

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
