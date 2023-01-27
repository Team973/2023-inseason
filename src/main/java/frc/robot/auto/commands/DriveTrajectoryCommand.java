package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.shared.AutoCommand;
import frc.robot.shared.Conversions.Time;
import frc.robot.shared.RobotInfo.DriveConstants;
import frc.robot.subsystems.Drive;

public class DriveTrajectoryCommand extends AutoCommand {
    private boolean autoStarted = false;
    private double startTimeSeconds = 0.0;

    private final Drive m_drive;
    private final Trajectory m_trajectory;

  public DriveTrajectoryCommand(Drive drive, Trajectory trajectory) {
    m_drive = drive;
    m_trajectory = trajectory;
  }

  public void init() {
    // TODO Auto-generated method stub

  }

  public void run() {
    if (!autoStarted) {
        startTimeSeconds = Time.getSecTime();
        autoStarted = true;
  
      double dtSeconds = Time.getSecTime() - startTimeSeconds;
  
      var goal = m_trajectory.sample(dtSeconds);
    }
  }

  public boolean isCompleted() {
    // TODO Auto-generated method stub
    return false;
  }
}
