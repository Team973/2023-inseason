package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SetDrivePositionCommand extends AutoCommand {

  private final Drive m_drive;

  private final Pose2d m_pose;
  private final Rotation2d m_rotation;

  public SetDrivePositionCommand(Drive drive, Pose2d pose, Rotation2d rotation) {
    m_drive = drive;
    m_pose = pose;
    m_rotation = rotation;
  }

  @Override
  public void init() {
    m_drive.resetOdometry(m_pose, m_rotation);
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
  public void postComplete() {}
}
