package frc.robot.commands.commandgroups;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.galacticsearch.GalacticSearchRedB;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrivetrain;

public class CGGalaticSearchRedB extends ParallelCommandGroup {

  public CGGalaticSearchRedB(SwerveDrivetrain drivetrain, Intake intake) throws IOException {
        addCommands(
          new GalacticSearchRedB(drivetrain),
          new WaitCommand(0.1).andThen(new SetIntakeSpeed(intake, 0.75).withTimeout(2.6)))
        ;
    }

}
