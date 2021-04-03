package frc.robot.commands.auto;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.subsystems.SwerveDrivetrain;

public class PathweaverTrajectoryCommand extends TrajectoryCommand {

    public PathweaverTrajectoryCommand(String path, Optional<Supplier<Rotation2d>> desiredRotation, Boolean shouldUpdate, double thetaP, SwerveDrivetrain drivetrain) throws IOException {
        super(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(path)),
                desiredRotation, shouldUpdate, thetaP, drivetrain);
    }

    public PathweaverTrajectoryCommand(String path, Boolean shouldUpdate, double thetaP, SwerveDrivetrain drivetrain) throws IOException {
        this(path, Optional.empty(), shouldUpdate, thetaP, drivetrain);
    }

    public PathweaverTrajectoryCommand(String path, SwerveDrivetrain drivetrain) throws IOException {
        this(path, false, 1, drivetrain);
    }

}
