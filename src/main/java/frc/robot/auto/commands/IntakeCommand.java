package frc.robot.auto.commands;

import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.GlobalState;

import lombok.AllArgsConstructor;

@AllArgsConstructor
public class IntakeCommand extends AutoCommand {
  private final Superstructure m_superstructure;
  private final IntakeState m_state;
  private final boolean m_autoStow;
  private final double m_timeout;

  @Override
  public void init() {
    setTargetMsec(m_timeout);
    m_superstructure.setDesiredIntakeState(m_state);
  }

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    if (hasElapsed()) {
      return true;
    }

    if (m_state == IntakeState.In) {
      return m_superstructure.isHasGamePiece();
    }

    // TODO: Enable once we have a cone banner sensor
    // else if (m_state == IntakeState.Out) {
    //   return !m_superstructure.isHasGamePiece();
    // }

    return false;
  }

  @Override
  public void postComplete(boolean interrupted) {
    if (m_state == IntakeState.Out) {
      if (m_autoStow) {
        m_superstructure.setDesiredGlobalState(GlobalState.PostScore);
      } else {
        m_superstructure.setDesiredIntakeState(IntakeState.Neutral);
        Superstructure.setCurrentGamePiece(GamePiece.None);
      }
    } else if (m_state == IntakeState.In) {
      m_superstructure.setDesiredIntakeState(IntakeState.Hold);
      if (m_autoStow) {
        m_superstructure.setDesiredGlobalState(GlobalState.Stow);
      }
    }
  }
}
