package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Wrist.WristPreset;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Superstructure implements Subsystem {

  private final Wrist m_wrist;
  private final Elevator m_elevator;

  public void dashboardUpdate() {}

  public void update() {
    if (m_elevator.getHeight() != m_elevator.getPreset().getValue()) {
      m_wrist.setPreset(WristPreset.PreStow);
    }
  }

  public void reset() {}
}
