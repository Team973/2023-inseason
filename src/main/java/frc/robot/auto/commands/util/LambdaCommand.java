package frc.robot.auto.commands.util;

import frc.robot.shared.AutoCommand;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class LambdaCommand extends AutoCommand {
  private final Runnable m_lambda;
  private boolean m_ran = false;

  public void init() {}

  public void run() {
    if (!m_ran) {
      m_lambda.run();
      m_ran = true;
    }
  }

  public boolean isCompleted() {
    return true;
  }

  public void postComplete(boolean interrupted) {}
}
