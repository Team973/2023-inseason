package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;

import lombok.AllArgsConstructor;

@AllArgsConstructor
public class SetCurrentGamePieceCommand extends AutoCommand {
  private final GamePiece m_gamePiece;

  @Override
  public void init() {
    Superstructure.setCurrentGamePiece(m_gamePiece);
  }

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return true;
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
