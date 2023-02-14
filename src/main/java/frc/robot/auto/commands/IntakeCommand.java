package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.subsystems.Claw.IntakeState;

public class IntakeCommand extends AutoCommand {
  private final Claw m_claw;

  private final IntakeState m_state;

  private final GamePiece m_gamePiece;

  private final double m_timeout;

  public IntakeCommand(
      Claw claw, IntakeState state, GamePiece gamePiece, double preset, double timeout) {
    m_claw = claw;
    m_state = state;
    m_gamePiece = gamePiece;
    m_timeout = timeout;
  }

  @Override
  public void init() {
    setTargetMsec(m_timeout);
    m_claw.setCurrentGamePiece(m_gamePiece);

    m_claw.setIntakeState(m_state);
  }

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return hasElapsed();
  }

  @Override
  public void postComplete() {
    m_claw.setIntakeState(IntakeState.Neutral);
  }
}
