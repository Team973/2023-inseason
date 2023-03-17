package frc.robot.auto.commands;

import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.shared.AutoCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;

public class ScorePreload extends AutoCommand {
  public ScorePreload(GamePiece gamePiece, Claw claw, Wrist wrist, Elevator elevator) {
    new SequentialCommand(
        new SetCurrentGamePieceCommand(gamePiece),
        new IntakeCommand(claw, IntakeState.In, 200),
        new WristPresetCommand(wrist, WristPreset.Offset, 10.0, 500),
        new ElevatorPresetCommand(elevator, Elevator.Preset.High, 4000),
        new WristPresetCommand(wrist, WristPreset.High, 1.0, 2000),
        new IntakeCommand(claw, IntakeState.Out, 500),
        new WaitCommand(200),
        new SetCurrentGamePieceCommand(GamePiece.None));
  }

  @Override
  public void init() {}

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return true;
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
