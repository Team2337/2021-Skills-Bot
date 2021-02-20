package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetDrivePosition extends InstantCommand {
    private final SwerveDrivetrain drivetrain;

    public ResetDrivePosition(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetDriveEncoders();
    }
  
}
