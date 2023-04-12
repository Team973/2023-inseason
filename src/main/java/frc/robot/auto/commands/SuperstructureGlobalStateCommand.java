package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GlobalState;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class SuperstructureGlobalStateCommand extends AutoCommand {
  private final Superstructure m_superstructure;
  private final GlobalState m_globalState;
  private final int m_timeout;

  @Override
  public void init() {
    setTargetMsec(m_timeout);
    m_superstructure.setDesiredGlobalState(m_globalState);
  }

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return m_superstructure.isAtTarget();
  }

  public void postComplete(boolean interrupted) {}
}
