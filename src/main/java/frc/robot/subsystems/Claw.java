package frc.robot.subsystems;

import frc.robot.shared.Subsystem;

public class Claw implements Subsystem {
  private ClampState m_clampState;
  private IntakeState m_intakeState;
 
  private enum ClampState {
    Open,
    Closed
  }

  private enum IntakeState {
    In,
    Out,
    Neutral
  }

  public Claw() {
    m_clampState = ClampState.Open;
    m_intakeState = IntakeState.Neutral;
  }

  public void open() {
    m_clampState = ClampState.Open;
  }

  public void close() {
    m_clampState = ClampState.Closed;
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
    switch (m_clampState) {
      case Open:
        break;
      case Closed:
        break;
      default:
        break;
    }
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
    m_clampState = ClampState.Open;
    m_intakeState = IntakeState.Neutral;
  }
}
