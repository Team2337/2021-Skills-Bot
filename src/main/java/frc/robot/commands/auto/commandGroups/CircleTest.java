package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveWithJoystickInputs;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

public class CircleTest extends SequentialCommandGroup {


    public CircleTest(SwerveDrivetrain drivetrain, Pigeon pigeon) {
        addCommands(
        new DriveWithJoystickInputs(drivetrain, .4, 0.0, 60.0, 0.0, 0, pigeon),  // Drive straight 60 inches
        new DriveWithJoystickInputs(drivetrain, .4, .5, 108.0, 45, 0, pigeon), //rotate 58 inches
        new DriveWithJoystickInputs(drivetrain, .2, 0, 138, 0, 0, pigeon), //Drive straight 30 inches
        new DriveWithJoystickInputs(drivetrain, 0, 0, 132, 0, 0, pigeon)  // stop
        );
    }
}