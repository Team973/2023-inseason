package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class PathPlannerTrajectoryCommand extends AutoCommand {
  private final Drive m_drive;

  @Getter private final PathPlannerTrajectory m_path;

  private final SetDrivePositionCommand m_positionCommand;
  private final DriveTrajectoryCommand m_trajectoryCommand;

  public PathPlannerTrajectoryCommand(Drive drive, PathPlannerTrajectory path) {
    m_drive = drive;
    m_path =
        PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
    m_positionCommand = new SetDrivePositionCommand(m_drive, m_path.getInitialHolonomicPose());
    m_trajectoryCommand = new DriveTrajectoryCommand(m_drive, m_path);
  }

  public PathPlannerTrajectoryCommand(
      Drive drive, String pathFile, PathConstraints constraints, boolean reverse) {
    this(drive, PathPlanner.loadPath(pathFile, constraints, reverse));
  }

  public void init() {
    m_positionCommand.init();
    m_trajectoryCommand.init();
  }

  public void run() {
    m_trajectoryCommand.run();
  }

  public boolean isCompleted() {
    return m_trajectoryCommand.isCompleted();
  }

  public void postComplete(boolean interrupted) {}
}
