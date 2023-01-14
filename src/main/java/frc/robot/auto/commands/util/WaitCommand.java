package frc.robot.auto.commands.util;

import frc.robot.shared.AutoCommand;

public class WaitCommand extends AutoCommand {
  private final double m_targetMsec;

  public WaitCommand(double targetMsec) {
    this.m_targetMsec = targetMsec;
  }

  public void init() {
    setTargetMsec(m_targetMsec);
  }

  public void run() {}

  public boolean isCompleted() {
    return hasElapsed();
  }
}
