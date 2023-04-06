package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristPreset;
import frc.robot.subsystems.Wrist.WristState;

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
    HpLoad,
    FloorLoad
  }

  @Getter @Setter private GlobalState m_globalState;
  private GlobalState m_lastElevatorState = m_globalState;
  private GlobalState m_lastWristState = m_globalState;
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
      case HpLoad:
        elevatorPreset = Elevator.Preset.Hp;
        wristPreset = WristPreset.Hp;
        break;
      case FloorLoad:
        elevatorPreset = Elevator.Preset.Floor;
        wristPreset = WristPreset.Floor;
        break;
      default:
        elevatorPreset = Elevator.Preset.Stow;
        wristPreset = WristPreset.Stow;
        break;
    }

    if ((Math.abs(elevatorPreset.getValue() - m_elevator.getHeight()) >= 5.0)
        && (m_wrist.getCurrentAngleDegrees() > 0.0)) {
      wristPreset = WristPreset.PreStow;
    } else if (m_globalState != m_lastWristState && m_wrist.getState() == WristState.ClosedLoop) {
      m_wrist.setPreset(wristPreset);
      m_lastWristState = m_globalState;
    } /*else if (wristPreset != WristPreset.PreStow
          && Math.abs(elevatorPreset.getValue() - m_elevator.getHeight()) < 5.0) {
        m_wrist.setPreset(wristPreset);
      }*/

    if (m_globalState != m_lastElevatorState
        && m_elevator.getElevatorState() == ElevatorState.ClosedLoop) {
      m_elevator.setPreset(elevatorPreset);
      m_lastElevatorState = m_globalState;
    }
    SmartDashboard.putString("Wrist preset", String.valueOf(wristPreset));
  }

  @Override
  public void reset() {
    setGlobalState(GlobalState.Stow);
  }

  @Override
  public void debugDashboardUpdate() {}
}
