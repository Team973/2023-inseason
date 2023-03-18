package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;

public class NoOpCommand extends AutoCommand {
  public void init() {}

  public void run() {}

  public boolean isCompleted() {
    return true;
  }

  public void postComplete(boolean interrupted) {}
}
