package frc.robot.auto.commands;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GlobalState;

public class TossCommand extends SuperstructureGlobalStateCommand {

  public TossCommand(Superstructure superstructure, int timeout) {
    super(superstructure, GlobalState.Toss, timeout);
  }

  @Override
  public boolean isCompleted() {
    return hasElapsed();
  }
}
