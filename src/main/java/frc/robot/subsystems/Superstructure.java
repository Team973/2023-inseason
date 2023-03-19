package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Elevator.ElevatorState;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Superstructure implements Subsystem {

  private final Wrist m_wrist;
  private final Elevator m_elevator;

  private boolean m_joystickPressed;

  public void joystickPressed(boolean pressed) {
    m_joystickPressed = pressed;
  }

  public void dashboardUpdate() {}

  public void update() {
    if (!m_wrist.isCollisionFree()) {
      m_elevator.setElevatorState(ElevatorState.WaitForWrist);
    } else if (!m_joystickPressed) {
      m_elevator.setElevatorState(ElevatorState.ClosedLoop);
    }
  }

  public void reset() {}
}
