package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Wrist.WristPreset;

public class Superstructure implements Subsystem {

  private final Wrist m_wrist = new Wrist();
  private final Elevator m_elevator = new Elevator();

  private boolean m_isStowed = true;

  public void wristStow() {
    if (!m_isStowed) {
      m_wrist.setPreset(WristPreset.PreStow);
      if (m_elevator.getBottomHall()) {
        m_wrist.setPreset(WristPreset.Stow);
        m_isStowed = true;
      }
    }
  }

  public void dashboardUpdate() {}

  public void update() {
    if (!m_elevator.getBottomHall()) {
      m_isStowed = false;
    }
  }

  public void reset() {}
}
