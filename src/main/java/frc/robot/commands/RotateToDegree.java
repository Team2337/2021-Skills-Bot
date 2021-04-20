package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

public class RotateToDegree extends PIDCommand {
    private final SwerveDrivetrain drivetrain;
    private final Pigeon pigeon;
    private final double degree;
    private int i = 0;
    private boolean done = false;
    public RotateToDegree(double degree, SwerveDrivetrain drivetrain, Pigeon pigeon) {
        super(
            new PIDController(0.009,0,0.0002),
            pigeon::getYaw,
            degree,
            output -> drivetrain.calculateJoystickInput(0, 0, Utilities.constraintOutput(output, 0.3), false),
            drivetrain
        );
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.pigeon = pigeon;
        this.degree = degree;
        getController().enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
      m_useOutput.accept(
          m_controller.calculate(m_measurement.getAsDouble(), m_setpoint.getAsDouble()));
          if(Utilities.withinTolerance(degree, pigeon.getYaw(), 1) && i <= 50) {
              i++;
              System.out.println(i);
            } else if(i > 50) {
                done = true;
          } else {
              i = 0;
          }
          SmartDashboard.putBoolean("withinTolerance", Utilities.withinTolerance(degree, pigeon.getYaw(), 1));
    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.stopAngleMotors();
        drivetrain.stopDriveMotors();
    }

    @Override
    public boolean isFinished() {
        return done;
        //return getController().atSetpoint();

    }
    
}
