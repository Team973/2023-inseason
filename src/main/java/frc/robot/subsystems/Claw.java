package frc.robot.subsystems;

import frc.robot.shared.Subsystem;

public class Claw implements Subsystem {
  private ClampState m_clampState;
  private IntakeState m_intakeState;

  public Claw() {}

  public void open() {
    m_clampState = ClampState.OPEN;
  }

  public void close() {
    m_clampState = ClampState.CLOSED;
  }

  public void intake() {
    m_intakeState = IntakeState.IN;
  }

  public void outtake() {
    m_intakeState = IntakeState.OUT;
  }

  public void stopIntake() {
    m_intakeState = IntakeState.NEUTRAL;
  }

  private enum ClampState {
    OPEN,
    CLOSED,
  }

  private enum IntakeState {
    IN,
    OUT,
    NEUTRAL
  }

  public void update() {
    switch (m_clampState) {
      case OPEN:
        break;
      case CLOSED:
        break;
      default:
        break;
    }
    switch (m_intakeState) {
      case IN:
        break;
      case OUT:
        break;
      case NEUTRAL:
        break;
      default:
        break;
    }
  }

  public void reset() {
    m_clampState = ClampState.OPEN;
    m_intakeState = IntakeState.NEUTRAL;
  }
}
