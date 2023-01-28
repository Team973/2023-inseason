package frc.robot.subsystems;

import frc.robot.shared.Subsystem;

import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Intake implements Subsystem {
  @Setter private IntakeState m_intakeState;

  @Setter @Getter private GamePiece m_currentGamePiece;

  public enum GamePiece {
    Cube,
    Cone
  }

  public enum IntakeState {
    In,
    Out,
    Neutral
  }

  public Intake() {
    m_intakeState = IntakeState.Neutral;
  }

  public void intake() {
    m_intakeState = IntakeState.In;
  }

  public void outtake() {
    m_intakeState = IntakeState.Out;
  }

  public void stopIntake() {
    m_intakeState = IntakeState.Neutral;
  }

  public void update() {
    switch (m_intakeState) {
      case In:
        break;
      case Out:
        break;
      case Neutral:
        break;
      default:
        break;
    }
  }

  public void reset() {
    m_intakeState = IntakeState.Neutral;
  }
}
