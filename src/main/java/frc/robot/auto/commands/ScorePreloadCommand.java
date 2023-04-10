package frc.robot.auto.commands;

import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.GlobalState;

public class ScorePreloadCommand extends AutoCommand {
  AutoCommand m_command;

  public ScorePreloadCommand(
      GamePiece gamePiece, GlobalState globalState, Superstructure superstructure) {
    m_command =
        new ConcurrentCommand(
            new SetCurrentGamePieceCommand(gamePiece),
            new IntakeCommand(superstructure, IntakeState.In, false, 4000),
            new SequentialCommand(
                new SuperstructureGlobalStateCommand(superstructure, globalState, 2000),
                new WaitCommand(800),
                new IntakeCommand(superstructure, IntakeState.Out, true, 800)));
  }

  @Override
  public void init() {
    m_command.init();
  }

  @Override
  public void run() {
    m_command.run();
  }

  @Override
  public boolean isCompleted() {
    return m_command.isCompleted();
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
