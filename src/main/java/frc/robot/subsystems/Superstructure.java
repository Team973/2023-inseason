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

  @Override
  public void dashboardUpdate() {
    // TODO Auto-generated method stub

  }

  @Override
  public void update() {
    switch (m_globalState) {
      case ScoreHigh:
        m_elevator.setPreset(Elevator.Preset.High);
        m_wrist.setPreset(WristPreset.High);
        break;
      case ScoreMid:
        m_elevator.setPreset(Elevator.Preset.Mid);
        m_wrist.setPreset(WristPreset.Mid);
        break;
      case ScoreLow:
        m_elevator.setPreset(Elevator.Preset.Hybrid);
        m_wrist.setPreset(WristPreset.Hybrid);
        break;
      case Stow:
        m_elevator.setPreset(Elevator.Preset.Stow);
        m_wrist.setPreset(WristPreset.Stow);
        break;
      case HpLoad:
        m_elevator.setPreset(Elevator.Preset.Hp);
        m_wrist.setPreset(WristPreset.HP);
        m_claw.setIntakeState(IntakeState.In);
        break;
      case FloorLoad:
        m_elevator.setPreset(Elevator.Preset.Floor);
        m_wrist.setPreset(WristPreset.Floor);
        m_claw.setIntakeState(IntakeState.In);
        break;
    }
  }

  @Override
  public void reset() {}
}
