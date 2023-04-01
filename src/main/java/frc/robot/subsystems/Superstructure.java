package frc.robot.subsystems;

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
  /** Game Piece options. */
  public enum GamePiece {
    Cube,
    Cone,
    None
  }

  /** Robot global state. */
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

  public void setPresets(Elevator.Preset elevatorPreset) {
    m_elevator.setPreset(elevatorPreset);
    String wristPreset = elevatorPreset.toString();
    if (Math.abs(elevatorPreset.getValue() - m_elevator.getHeight()) < 0.5) {
      m_wrist.setPreset(WristPreset.valueOf(wristPreset));
    } else {
      m_wrist.setPreset(WristPreset.Offset);
    }
  }

  @Override
  public void dashboardUpdate() {}

  @Override
  public void update() {
    switch (m_globalState) {
      case ScoreHigh:
        setPresets(Elevator.Preset.High);
        break;
      case ScoreMid:
        setPresets(Elevator.Preset.Mid);
        break;
      case ScoreLow:
        setPresets(Elevator.Preset.Hybrid);
        break;
      case Stow:
        setPresets(Elevator.Preset.Stow);
        break;
      case HpLoad:
        setPresets(Elevator.Preset.Hp);
        m_claw.setIntakeState(IntakeState.In);
        break;
      case FloorLoad:
        setPresets(Elevator.Preset.Floor);
        m_claw.setIntakeState(IntakeState.In);
        break;
    }
  }

  @Override
  public void reset() {}
}
