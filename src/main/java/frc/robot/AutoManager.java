package frc.robot;

import frc.robot.auto.modes.CenterPreloadAndCharge;
import frc.robot.auto.modes.NoAuto;
import frc.robot.auto.modes.PreloadAndCharge;
import frc.robot.auto.modes.PreloadPickupCharge;
import frc.robot.auto.modes.Test;
import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

public class AutoManager {
  private AutoCommand m_currentMode;

  public enum AutoMode {
    Test,
    PreloadAndCharge,
    PreloadPickupCharge,
    CenterPreloadAndCharge,
    NoAuto
  }

  public enum AutoSide {
    Left,
    Right,
    Center
  }

  private final AutoCommand m_test;
  private final AutoCommand m_preloadAndCharge;
  private final AutoCommand m_preloadPickupCharge;
  private final AutoCommand m_centerPreloadAndCharge;
  private final AutoCommand m_noAuto;

  public AutoManager(Claw claw, Elevator elevator, Drive drive) {
    m_test = new Test(drive);
    m_preloadAndCharge = new PreloadAndCharge(drive, claw, elevator);
    m_preloadPickupCharge = new PreloadPickupCharge(drive, claw, elevator);
    m_centerPreloadAndCharge = new CenterPreloadAndCharge(drive, claw, elevator);
    m_noAuto = new NoAuto();
  }

  public void run() {
    m_currentMode.run();
  }

  public void init() {
    m_currentMode.init();
  }

  public void selectAuto(AutoMode mode) {
    switch (mode) {
      case Test:
        m_currentMode = m_test;
        break;
      case PreloadAndCharge:
        m_currentMode = m_preloadAndCharge;
        break;
      case PreloadPickupCharge:
        m_currentMode = m_preloadPickupCharge;
        break;
      case CenterPreloadAndCharge:
        m_currentMode = m_centerPreloadAndCharge;
        break;
      case NoAuto:
        m_currentMode = m_noAuto;
        break;
    }
  }
}
