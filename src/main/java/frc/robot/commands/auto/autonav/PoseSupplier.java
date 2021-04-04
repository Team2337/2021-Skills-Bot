package frc.robot.commands.auto.autonav;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import java.util.List;

interface PoseSupplier {
  public List<Pose2d> getPoses();
}