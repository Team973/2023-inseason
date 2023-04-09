package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Wrist.WristPreset;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
@RequiredArgsConstructor
public class Superstructure implements Subsystem {
  private static final Rotation2d STOW_DANGER_ANGLE = Rotation2d.fromDegrees(0.0);
  private static final Rotation2d SCORE_DANGER_ANGLE = Rotation2d.fromDegrees(-40.0);

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
    Toss,
    Score,
    PostScore,
    Manual
  }

  @Getter @Setter private GlobalState m_globalState = GlobalState.Stow;
  @Getter @Setter private static GamePiece m_currentGamePiece = GamePiece.None;
  @Getter @Setter IntakeState m_desiredIntakeState = IntakeState.Neutral;

  private final Elevator m_elevator;
  private final Wrist m_wrist;
  private final Claw m_claw;

  public void dashboardUpdate() {}

  public void update() {
    Elevator.Preset elevatorPreset = m_elevator.getPreset();
    WristPreset wristPreset = m_wrist.getPreset();

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
      case Toss:
        elevatorPreset = Elevator.Preset.Floor;
        wristPreset = WristPreset.Hybrid;

        if (Math.abs(m_wrist.getVelocity().getDegrees()) > 80.0) {
          setDesiredIntakeState(IntakeState.Out);
        } else if (m_claw.getIntakeState() == IntakeState.Out) {
          setGlobalState(GlobalState.PostScore);
        }
        break;
      case Score:
        setDesiredIntakeState(IntakeState.Out);
        break;
      case PostScore:
        setDesiredIntakeState(IntakeState.Neutral);
        setCurrentGamePiece(GamePiece.None);
        setGlobalState(GlobalState.Stow);
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

    boolean wristInStowDangerZone =
        wristPreset.getConePreset().getDegrees() > STOW_DANGER_ANGLE.getDegrees()
            || wristPreset.getCubePreset().getDegrees() > STOW_DANGER_ANGLE.getDegrees();
    boolean elevatorInStowDangerZone =
        (elevatorPreset.getValue() > 14.78 && m_elevator.getHeight() < 14.78)
            || (elevatorPreset.getValue() < 14.78 && m_elevator.getHeight() > 14.78);

    boolean wristInScoreDangerZone =
        wristPreset.getConePreset().getDegrees() < SCORE_DANGER_ANGLE.getDegrees()
            || wristPreset.getCubePreset().getDegrees() < SCORE_DANGER_ANGLE.getDegrees();
    boolean elevatorInScoreDangerZone =
        (elevatorPreset.getValue() > 15.82 && m_elevator.getHeight() < 15.82)
            || (elevatorPreset.getValue() < 22.59 && m_elevator.getHeight() > 22.59)
            || (elevatorPreset.getValue() > 22.59 && m_elevator.getHeight() < 22.59)
            || (elevatorPreset.getValue() < 15.82 && m_elevator.getHeight() > 15.82);

    if (m_globalState != GlobalState.Manual) {
      if (wristInStowDangerZone && elevatorInStowDangerZone) {
        wristPreset = WristPreset.PreStow;
      } else if (wristInScoreDangerZone && elevatorInScoreDangerZone) {
        wristPreset = WristPreset.PreScore;
      }
    }
    m_wrist.setPreset(wristPreset);
    m_elevator.setPreset(elevatorPreset);
    m_claw.setIntakeState(m_desiredIntakeState);
  }

  public void reset() {
    setGlobalState(GlobalState.Stow);
  }

  public void debugDashboardUpdate() {
    SmartDashboard.putString("Global State", String.valueOf(m_globalState));
    SmartDashboard.putString("Intake State", String.valueOf(m_desiredIntakeState));
  }
}
