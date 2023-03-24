package frc.robot.auto.commands;

import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.shared.AutoCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;

public class ScorePreloadCommand extends AutoCommand {
  AutoCommand m_command;

  public ScorePreloadCommand(
      GamePiece gamePiece,
      Elevator.Preset elevatorPreset,
      WristPreset wristPreset,
      Claw claw,
      Wrist wrist,
      Elevator elevator) {
    m_command =
        new ConcurrentCommand(
            new SetCurrentGamePieceCommand(gamePiece),
            new IntakeCommand(claw, IntakeState.In, 3000),
            new SequentialCommand(
                new WristPresetCommand(wrist, WristPreset.Offset, 10.0, 500),
                new ElevatorPresetCommand(elevator, elevatorPreset, 4000),
                new WristPresetCommand(wrist, wristPreset, 1.0, 2000),
                new IntakeCommand(claw, IntakeState.Out, 500),
                new WaitCommand(200),
                new SetCurrentGamePieceCommand(GamePiece.None)));
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
