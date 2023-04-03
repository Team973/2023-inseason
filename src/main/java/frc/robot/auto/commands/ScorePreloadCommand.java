package frc.robot.auto.commands;

import frc.robot.auto.commands.util.ConcurrentCommand;
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
            new IntakeCommand(claw, IntakeState.In, 4000),
            new SequentialCommand(
                new WristPresetCommand(wrist, WristPreset.PreStow, 10.0, 100),
                new ElevatorPresetCommand(elevator, elevatorPreset, 2000),
                new WristPresetCommand(wrist, wristPreset, 1.0, 2000),
                new WaitCommand(800),
                new IntakeCommand(claw, IntakeState.Out, 800),
                new WaitCommand(200),
                new SetCurrentGamePieceCommand(GamePiece.None),
                new ElevatorPresetCommand(elevator, Elevator.Preset.HighOffset, 500)));
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
