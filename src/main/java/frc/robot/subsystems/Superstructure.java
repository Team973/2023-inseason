package frc.robot.subsystems;

import frc.robot.shared.Subsystem;
import frc.robot.subsystems.CANdleManager.LightState;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Wrist.WristPreset;

public class Superstructure implements Subsystem {

  private final Wrist m_wrist = new Wrist();
  private final Elevator m_elevator = new Elevator();
  private final Claw m_claw = new Claw();
  private final CANdleManager m_candleManager = new CANdleManager();

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

    if (m_claw.getIntakeState() == IntakeState.In && m_claw.isHasGamePiece()) {
      m_candleManager.setLightState(LightState.GotIt);

      wristStow();
      m_elevator.setPreset(Elevator.Preset.Stow);
    }
  }

  public void reset() {}
}
