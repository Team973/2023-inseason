package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.shared.LimelightHelpers;
import frc.robot.subsystems.Drive;

public class ResetPositionCommand extends AutoCommand {

  private Drive m_drive;

  public ResetPositionCommand(Drive drive) {
    m_drive = drive;
  }

  @Override
  public void init() {
    var pose = LimelightHelpers.getBotPose2d_wpiBlue("");
    m_drive.resetOdometry(pose);
  }

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return true;
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
