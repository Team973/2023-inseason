package frc.robot.auto.commands;

import frc.robot.auto.TrajectoryManager.Determinator;
import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;

import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class PathPlannerTrajectoryCommand extends AutoCommand {
  private final Drive m_drive;

  private final Determinator m_determinator;
  private DriveTrajectoryCommand m_trajectoryCommand;
  private final boolean m_doZero;

  public PathPlannerTrajectoryCommand(Drive drive, Determinator determinator) {
    m_drive = drive;
    m_determinator = determinator;
    m_trajectoryCommand = null;
    m_doZero = true;
  }

  public PathPlannerTrajectoryCommand(Drive drive, boolean doZero, Determinator determinator) {
    m_drive = drive;
    m_determinator = determinator;
    m_trajectoryCommand = null;
    m_doZero = doZero;
  }

  public void init() {
    var path = m_determinator.determine();
    m_trajectoryCommand = new DriveTrajectoryCommand(m_drive, path);
    m_trajectoryCommand.init();
    if (m_doZero) {
      m_drive.resetOdometry(path.getInitialHolonomicPose());
    }
  }

  public void run() {
    if (m_trajectoryCommand != null) {
      m_trajectoryCommand.run();
    }
  }

  public boolean isCompleted() {
    return m_trajectoryCommand != null ? m_trajectoryCommand.isCompleted() : true;
  }

  public void postComplete(boolean interrupted) {}
}
