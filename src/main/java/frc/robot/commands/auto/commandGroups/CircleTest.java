package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveWithJoystickInputs;
import frc.robot.subsystems.SwerveDrivetrain;

public class CircleTest extends SequentialCommandGroup {


    public CircleTest(SwerveDrivetrain drivetrain) {
        addCommands(
        new DriveWithJoystickInputs(drivetrain, .4, 60.0, 0),
        new DriveWithJoystickInputs(drivetrain, .4, 108.0, 45),
        new DriveWithJoystickInputs(drivetrain, .2, 132, 0),
        new DriveWithJoystickInputs(drivetrain, 0, 132, 0)
        );
    }
}