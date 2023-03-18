package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Elevator.ElevatorState;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Superstructure implements Subsystem {

  private final Wrist m_wrist;
  private final Elevator m_elevator;

  public void dashboardUpdate() {}

  public void update() {
    if (!m_wrist.isCollisionFree()) {
      m_elevator.setElevatorState(ElevatorState.WaitForWrist);
    } else {
      m_elevator.setElevatorState(ElevatorState.ClosedLoop);
    }
  }

  public void reset() {}
}
