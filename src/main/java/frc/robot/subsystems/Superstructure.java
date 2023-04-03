package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Elevator.Preset;
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

  @Override
  public void dashboardUpdate() {}

  @Override
  public void update() {
    Elevator.Preset elevatorPreset;
    WristPreset wristPreset;

    switch (m_globalState) {
      case ScoreHigh:
        elevatorPreset = Preset.High;
        wristPreset = WristPreset.High;
        break;
      case ScoreMid:
        elevatorPreset = Preset.Mid;
        wristPreset = WristPreset.Mid;
        break;
      case ScoreLow:
        elevatorPreset = Preset.Hybrid;
        wristPreset = WristPreset.Hybrid;
        break;
      case Stow:
        elevatorPreset = Preset.Stow;
        wristPreset = WristPreset.Stow;
        break;
      case HpLoad:
        elevatorPreset = Preset.Hp;
        wristPreset = WristPreset.Hp;
        break;
      case FloorLoad:
        elevatorPreset = Preset.Floor;
        wristPreset = WristPreset.Floor;
        break;
      default:
        elevatorPreset = Preset.Stow;
        wristPreset = WristPreset.Stow;
        break;
    }

    if ((Math.abs(elevatorPreset.getValue() - m_elevator.getHeight()) >= 5.0)
        && (m_wrist.getCurrentAngleDegrees() > 0.0)) {
      wristPreset = WristPreset.PreStow;
    }

    m_elevator.setPreset(elevatorPreset);
    m_wrist.setPreset(wristPreset);
  }

  @Override
  public void reset() {
    setGlobalState(GlobalState.Stow);
  }
}
