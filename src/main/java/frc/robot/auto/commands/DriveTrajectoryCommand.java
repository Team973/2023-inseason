package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.shared.Conversions;
import frc.robot.subsystems.Drive;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.trajectory.Trajectory;

public class DriveTrajectoryCommand extends AutoCommand {
  private double m_startTimeSeconds = 0.0;

  private final Drive m_drive;
  private final Trajectory m_trajectory;

  public DriveTrajectoryCommand(Drive drive, Trajectory trajectory) {
    m_drive = drive;
    m_trajectory = trajectory;
  }

  public void init() {
    m_startTimeSeconds = Conversions.Time.getSecTime();
  }

  public void run() {
    double dtSeconds = Conversions.Time.getSecTime() - m_startTimeSeconds;

    var goal = (PathPlannerState) m_trajectory.sample(dtSeconds);
    m_drive.driveInput(goal, goal.holonomicRotation);
  }

  public boolean isCompleted() {
    double dtSeconds = Conversions.Time.getSecTime() - m_startTimeSeconds;
    return m_trajectory.getTotalTimeSeconds() <= dtSeconds;
  }

  public void postComplete(boolean interrupted) {}
}
