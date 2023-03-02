package frc.robot;

import frc.robot.auto.commands.TrajectoryManager;
import frc.robot.auto.modes.NoAuto;
import frc.robot.auto.modes.OneCone;
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
    OneCone,
    PreloadAndCharge,
    PreloadPickupCharge,
    NoAuto
  }

  public enum AutoSide {
    Left,
    Right,
    Center
  }

  private final AutoCommand m_test;
  private final AutoCommand m_oneCone;
  private final AutoCommand m_preloadAndCharge;
  private final AutoCommand m_preloadPickupCharge;
  private final AutoCommand m_noAuto;

  public AutoManager(
      Claw claw, Elevator elevator, Drive drive, TrajectoryManager trajectoryManager) {
    m_test = new Test(drive, trajectoryManager);
    m_oneCone = new OneCone(drive, claw, elevator, trajectoryManager);
    m_preloadAndCharge = new PreloadAndCharge(drive, claw, elevator);
    m_preloadPickupCharge = new PreloadPickupCharge(drive, claw, elevator);
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
      case OneCone:
        m_currentMode = m_oneCone;
        break;
      case PreloadAndCharge:
        m_currentMode = m_preloadAndCharge;
        break;
      case PreloadPickupCharge:
        m_currentMode = m_preloadPickupCharge;
        break;
      case NoAuto:
        m_currentMode = m_noAuto;
        break;
    }
  }
}
