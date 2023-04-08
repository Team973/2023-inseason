package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Wrist.WristPreset;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    LoadHp,
    LoadFloor,
    Manual
  }

  @Getter @Setter private GlobalState m_globalState;
  private final Elevator m_elevator;
  private final Wrist m_wrist;

  @Override
  public void dashboardUpdate() {
    SmartDashboard.putString("globalState", String.valueOf(m_globalState));
  }

  @Override
  public void update() {
    Elevator.Preset elevatorPreset;
    WristPreset wristPreset;

    switch (m_globalState) {
      case ScoreHigh:
        elevatorPreset = Elevator.Preset.High;
        wristPreset = WristPreset.High;
        break;
      case ScoreMid:
        elevatorPreset = Elevator.Preset.Mid;
        wristPreset = WristPreset.Mid;
        break;
      case ScoreLow:
        elevatorPreset = Elevator.Preset.Hybrid;
        wristPreset = WristPreset.Hybrid;
        break;
      case Stow:
        elevatorPreset = Elevator.Preset.Stow;
        wristPreset = WristPreset.Stow;
        break;
      case LoadHp:
        elevatorPreset = Elevator.Preset.Hp;
        wristPreset = WristPreset.Hp;
        break;
      case LoadFloor:
        elevatorPreset = Elevator.Preset.Floor;
        wristPreset = WristPreset.Floor;
        break;
      case Manual:
        elevatorPreset = Elevator.Preset.Manual;
        wristPreset = WristPreset.Manual;
        break;
      default:
        elevatorPreset = Elevator.Preset.Stow;
        wristPreset = WristPreset.Stow;
        break;
    }

    /*   if () && (m_wrist.getCurrentAngleDegrees() > 0.0)) {
      wristPreset = WristPreset.PreStow;
    }*/
    m_wrist.setPreset(wristPreset);
    m_elevator.setPreset(elevatorPreset);
  }

  @Override
  public void reset() {
    setGlobalState(GlobalState.Stow);
  }

  @Override
  public void debugDashboardUpdate() {}
}
