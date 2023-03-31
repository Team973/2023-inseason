package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Wrist.WristPreset;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
@RequiredArgsConstructor
public class Superstructure implements Subsystem {
  public enum GlobalState {
    ScoreHigh,
    ScoreMid,
    ScoreLow,
    Stow,
    HpLoad,
    FloorLoad
  }

  @Getter @Setter private GlobalState m_globalState;
  private final Elevator m_elevator;
  private final Wrist m_wrist;
  private final Claw m_claw;
  private GamePiece m_currentGamePiece = GamePiece.None;

  @Override
  public void dashboardUpdate() {
    // TODO Auto-generated method stub

  }

  @Override
  public void update() {
    m_currentGamePiece = Robot.getCurrentGamePiece();
    switch (m_globalState) {
      case ScoreHigh:
        m_elevator.setPreset(Elevator.Preset.High);
        m_wrist.setPreset(WristPreset.High);
        if (m_claw.getIntakeState() == IntakeState.Out) {
          m_claw.setIntakeState(IntakeState.Neutral);
          m_currentGamePiece = GamePiece.None;
          m_wrist.setPreset(WristPreset.Stow);
          m_elevator.setPreset(Elevator.Preset.Stow);
        } else {
          if (m_claw.isHasGamePiece() && (m_wrist.getPreset() == WristPreset.Stow)) {
            m_wrist.setPreset(WristPreset.ConeRight);
          }
          m_claw.setIntakeState(IntakeState.Out);
        }
        break;
      case ScoreMid:
        m_elevator.setPreset(Elevator.Preset.Mid);
        m_wrist.setPreset(WristPreset.Mid);
        if (m_claw.getIntakeState() == IntakeState.Out) {
          m_claw.setIntakeState(IntakeState.Neutral);
          m_currentGamePiece = GamePiece.None;
          m_wrist.setPreset(WristPreset.Stow);
          m_elevator.setPreset(Elevator.Preset.Stow);
        } else {
          if (m_claw.isHasGamePiece() && (m_wrist.getPreset() == WristPreset.Stow)) {
            m_wrist.setPreset(WristPreset.ConeRight);
          }
          m_claw.setIntakeState(IntakeState.Out);
        }
        break;
      case ScoreLow:
        m_elevator.setPreset(Elevator.Preset.Hybrid);
        m_wrist.setPreset(WristPreset.Hybrid);
        if (m_claw.getIntakeState() == IntakeState.Out) {
          m_claw.setIntakeState(IntakeState.Neutral);
          m_currentGamePiece = GamePiece.None;
          m_wrist.setPreset(WristPreset.Stow);
          m_elevator.setPreset(Elevator.Preset.Stow);
        } else {
          if (m_claw.isHasGamePiece() && (m_wrist.getPreset() == WristPreset.Stow)) {
            m_wrist.setPreset(WristPreset.ConeRight);
          }
          m_claw.setIntakeState(IntakeState.Out);
        }
        break;
      case Stow:
        m_elevator.setPreset(Elevator.Preset.Stow);
        m_wrist.setPreset(WristPreset.Stow);
        break;
      case HpLoad:
        m_elevator.setPreset(Elevator.Preset.Hp);
        m_wrist.setPreset(WristPreset.HP);
        if (m_claw.getIntakeState() == IntakeState.Out) {
          m_claw.setIntakeState(IntakeState.Neutral);
          m_currentGamePiece = GamePiece.None;
          m_wrist.setPreset(WristPreset.Stow);
          m_elevator.setPreset(Elevator.Preset.Stow);
        } else {
          if (m_claw.isHasGamePiece() && (m_wrist.getPreset() == WristPreset.Stow)) {
            m_wrist.setPreset(WristPreset.ConeRight);
          }
          m_claw.setIntakeState(IntakeState.Out);
        }

        break;
      case FloorLoad:
        m_elevator.setPreset(Elevator.Preset.Floor);
        m_wrist.setPreset(WristPreset.Floor);
        if (m_claw.getIntakeState() == IntakeState.Out) {
          m_claw.setIntakeState(IntakeState.Neutral);
          m_currentGamePiece = GamePiece.None;
          m_wrist.setPreset(WristPreset.Stow);
          m_elevator.setPreset(Elevator.Preset.Stow);
        } else {
          if (m_claw.isHasGamePiece() && (m_wrist.getPreset() == WristPreset.Stow)) {
            m_wrist.setPreset(WristPreset.ConeRight);
          }
          m_claw.setIntakeState(IntakeState.Out);
        }

        break;
    }
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub

  }
}
