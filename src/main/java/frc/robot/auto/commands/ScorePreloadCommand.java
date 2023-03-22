package frc.robot.auto.commands;

import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;

public class ScorePreloadCommand extends AutoCommand {
  SequentialCommand m_command;

  public ScorePreloadCommand(GamePiece gamePiece, Claw claw, Wrist wrist, Elevator elevator) {
    m_command =
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
