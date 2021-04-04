package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveWithJoystickInputs;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrivetrain;

public class CircleTest2 extends SequentialCommandGroup {


    public CircleTest2(SwerveDrivetrain drivetrain, Pigeon pigeon) {
        addCommands(
        new DriveWithJoystickInputs(drivetrain, .4, 0.0, 91.2, 0, 0),  // Drive straight 60 inches
        new DriveWithJoystickInputs(drivetrain, .4, 0, 270.0, .35, 0), //rotate 58 inches
        new DriveWithJoystickInputs(drivetrain, .2, 0, 360, 0, 0), //Drive straight 30 inches
        new DriveWithJoystickInputs(drivetrain, 0, 0, 361, 0, 0)  // stop
        );
    }
}