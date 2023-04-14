package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Claw.IntakeState;
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
    Toss,
    Score,
    PostScore,
    Manual
  }

  @Getter @Setter private GlobalState m_desiredGlobalState = GlobalState.Stow;
  @Getter private GlobalState m_currentGlobalState;
  @Getter @Setter private static GamePiece m_currentGamePiece = GamePiece.None;
  @Getter @Setter IntakeState m_desiredIntakeState = IntakeState.Neutral;

  private final Elevator m_elevator;
  private final Wrist m_wrist;
  private final Claw m_claw;

  public void dashboardUpdate() {}

  public void update() {
    Elevator.Preset elevatorPreset = m_elevator.getPreset();
    WristPreset wristPreset = m_wrist.getPreset();

    switch (m_desiredGlobalState) {
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

        double releaseVelocity = m_currentGamePiece == GamePiece.Cone ? 100.0 : 80.0;

        if (Math.abs(m_wrist.getVelocity()) > releaseVelocity) {
          setDesiredIntakeState(IntakeState.Toss);
        } else if (m_claw.getIntakeState() == IntakeState.Toss) {
          setDesiredGlobalState(GlobalState.PostScore);
        }
        break;
      case Score:
        setDesiredIntakeState(IntakeState.Out);
        break;
      case PostScore:
        setDesiredIntakeState(IntakeState.Neutral);
        setCurrentGamePiece(GamePiece.None);
        setDesiredGlobalState(GlobalState.Stow);
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
        m_wrist.getTargetAngle() > 0.0 || m_wrist.getCurrentAngleDegrees() > 0.0;
    boolean elevatorInStowDangerZone =
        (m_elevator.getTargetHeight() > 14.78 && m_elevator.getHeight() < 14.78)
            || (m_elevator.getTargetHeight() < 14.78 && m_elevator.getHeight() > 14.78);

    boolean wristInScoreDangerZone =
        wristPreset.getConePreset() < -40.0 || wristPreset.getCubePreset() < -40.0;
    boolean elevatorInScoreDangerZone =
        (elevatorPreset.getValue() > 15.82 && m_elevator.getHeight() < 15.82)
            || (elevatorPreset.getValue() < 22.59 && m_elevator.getHeight() > 22.59)
            || (elevatorPreset.getValue() > 22.59 && m_elevator.getHeight() < 22.59)
            || (elevatorPreset.getValue() < 15.82 && m_elevator.getHeight() > 15.82);

    if (m_desiredGlobalState != GlobalState.Manual) {
      if (wristInStowDangerZone && elevatorInStowDangerZone) {
        wristPreset = WristPreset.PreStow;
      } else if (wristInScoreDangerZone && elevatorInScoreDangerZone) {
        wristPreset = WristPreset.PreScore;
      }
    }
    m_wrist.setPreset(wristPreset);
    m_elevator.setPreset(elevatorPreset);
    m_claw.setIntakeState(m_desiredIntakeState);

    m_currentGlobalState = m_desiredGlobalState;
  }

  public void reset() {
    setDesiredGlobalState(GlobalState.Stow);
  }

  public boolean isAtTarget() {
    return m_elevator.isAtTarget()
        && m_wrist.isAtTargetAngle()
        && m_desiredGlobalState == m_currentGlobalState;
  }

  public boolean isHasGamePiece() {
    // For auto: we want the most realtime state
    m_claw.checkForGamePiece();
    return m_claw.isHasGamePiece();
  }

  public void debugDashboardUpdate() {
    SmartDashboard.putString("Desired Global State", m_desiredGlobalState.toString());
    SmartDashboard.putString("Intake State", String.valueOf(m_desiredIntakeState));
  }
}
