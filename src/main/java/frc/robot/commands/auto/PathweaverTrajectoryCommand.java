package frc.robot.commands.auto;

import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.subsystems.SwerveDrivetrain;

public class PathweaverTrajectoryCommand extends TrajectoryCommand {
    public PathweaverTrajectoryCommand(String path, SwerveDrivetrain drivetrain) throws IOException {
        super(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(path)), drivetrain);
    }
}
