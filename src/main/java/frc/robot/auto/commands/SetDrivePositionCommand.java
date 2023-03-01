package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class SetDrivePositionCommand extends AutoCommand {

  private final Drive m_drive;

  private final Pose2d m_pose;

  public SetDrivePositionCommand(Drive drive, Pose2d pose) {
    m_drive = drive;
    m_pose = pose;
  }

  @Override
  public void init() {
    m_drive.resetOdometry(m_pose);
  }

  @Override
  public void run() {
    // Do nothing
  }

  @Override
  public boolean isCompleted() {
    return true;
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
