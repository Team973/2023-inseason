package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;

import lombok.AllArgsConstructor;

@AllArgsConstructor
public class IntakeCommand extends AutoCommand {
  private final Claw m_claw;

  private final IntakeState m_state;

  private final GamePiece m_gamePiece;

  private final double m_timeout;

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
    if (m_state == IntakeState.Out) {
      m_claw.setIntakeState(IntakeState.Neutral);
    } else if (m_state == IntakeState.In) {
      m_claw.setIntakeState(IntakeState.Hold);
    }
  }
}
