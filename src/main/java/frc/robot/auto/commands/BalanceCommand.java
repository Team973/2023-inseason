package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;

import lombok.NonNull;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class BalanceCommand extends AutoCommand {
  @NonNull private final Drive m_drive;
  private final double m_targetMsec;

  public void init() {
    setTargetMsec(m_targetMsec);
  }

  public void run() {
    m_drive.balanceDrive();
  }

  public boolean isCompleted() {
    return m_drive.getPigeon().isLevel();
  }

  public void postComplete(boolean interrupted) {}
}
