package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathPlannerTrajectoryCommand extends AutoCommand {
  private final Drive m_drive;

  private PathPlannerTrajectory m_path = PathPlanner.loadPath("Path", new PathConstraints(4, 3));

  private DriveTrajectoryCommand m_trajectoryCommand;

  public PathPlannerTrajectoryCommand(Drive drive) {
    m_drive = drive;

    m_trajectoryCommand = new DriveTrajectoryCommand(m_drive, m_path);
  }

  public void init() {
    m_trajectoryCommand.init();
  }

  public void run() {
    m_trajectoryCommand.run();
  }

  public boolean isCompleted() {
    return m_trajectoryCommand.isCompleted();
  }
}
