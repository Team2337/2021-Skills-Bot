package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class resetOdometry extends InstantCommand {
    private final SwerveDrivetrain drivetrain;

    public resetOdometry(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetModuleStates();
    }
  
}
