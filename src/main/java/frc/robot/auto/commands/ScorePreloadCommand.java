package frc.robot.auto.commands;

import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.GlobalState;

public class ScorePreloadCommand extends ConcurrentCommand {
  public ScorePreloadCommand(
      GamePiece gamePiece, GlobalState globalState, Superstructure superstructure) {
    super(
        new SetCurrentGamePieceCommand(gamePiece),
        new IntakeCommand(superstructure, IntakeState.In, false, 1500),
        new SequentialCommand(
            new SuperstructureGlobalStateCommand(superstructure, globalState, 3500),

            // TODO: Increase timeout once we have a cone banner sensor
            new IntakeCommand(superstructure, IntakeState.Out, true, 320)));
  }
}
