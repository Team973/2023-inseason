package frc.robot;

import java.util.Arrays;
import java.util.List;

import frc.robot.auto.modes.CenterPreloadAndCharge;
import frc.robot.auto.modes.MidLinkNoCharge;
import frc.robot.auto.modes.NoAuto;
import frc.robot.auto.modes.PreloadAndCharge;
import frc.robot.auto.modes.PreloadDoubleShootClose;
import frc.robot.auto.modes.PreloadDoubleShootFar;
import frc.robot.auto.modes.PreloadPickupCharge;
import frc.robot.auto.modes.PreloadPickupScoreCharge;
import frc.robot.auto.modes.Test;
import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class AutoManager {
  private AutoCommand m_currentMode;
  private final List<AutoMode> m_availableAutoModes =
      Arrays.asList(
          AutoMode.PreloadPickupScoreCharge,
          AutoMode.MidLinkNoCharge,
          AutoMode.PreloadPickupCharge,
          AutoMode.Test,
          AutoMode.PreloadAndCharge,
          AutoMode.CenterPreloadAndCharge,
          AutoMode.PreloadDoubleShootFar,
          AutoMode.PreloadDoubleShootClose,
          AutoMode.NoAuto);
  private int m_selectedMode = 0;

  public enum AutoMode {
    Test,
    PreloadAndCharge,
    PreloadPickupCharge,
    CenterPreloadAndCharge,
    PreloadPickupScoreCharge,
    MidLinkNoCharge,
    NoAuto,
    PreloadDoubleShootClose,
    PreloadDoubleShootFar,
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
  private final AutoCommand m_preloadPickupScoreCharge;
  private final AutoCommand m_midLinkNoCharge;
  private final AutoCommand m_preloadDoubleShootFar;
  private final AutoCommand m_preloadDoubleShootClose;

  public AutoManager(Claw claw, Elevator elevator, Drive drive, Wrist wrist) {
    m_test = new Test(claw, wrist, elevator);
    m_preloadAndCharge = new PreloadAndCharge(drive, claw, elevator, wrist);
    m_preloadPickupCharge = new PreloadPickupCharge(drive, claw, elevator, wrist);
    m_centerPreloadAndCharge = new CenterPreloadAndCharge(drive, claw, elevator, wrist);
    m_preloadPickupScoreCharge = new PreloadPickupScoreCharge(drive, claw, elevator, wrist);
    m_midLinkNoCharge = new MidLinkNoCharge(drive, claw, elevator, wrist);
    m_noAuto = new NoAuto();
    m_preloadDoubleShootFar = new PreloadDoubleShootFar(drive, claw, elevator, wrist);
    m_preloadDoubleShootClose = new PreloadDoubleShootClose(drive, claw, elevator, wrist);
  }

  public void increment() {
    m_selectedMode += 1;
  }

  public void decrement() {
    m_selectedMode -= 1;
  }

  public AutoMode getSelectedMode() {
    if (m_selectedMode >= m_availableAutoModes.size()) {
      m_selectedMode = 0;
    }
    if (m_selectedMode < 0) {
      m_selectedMode = m_availableAutoModes.size() - 1;
    }
    return m_availableAutoModes.get(m_selectedMode);
  }

  public void run() {
    m_currentMode.run();
  }

  public void init() {
    selectAuto(m_availableAutoModes.get(m_selectedMode));
    m_currentMode.init();
  }

  private void selectAuto(AutoMode mode) {
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
      case PreloadPickupScoreCharge:
        m_currentMode = m_preloadPickupScoreCharge;
        break;
      case MidLinkNoCharge:
        m_currentMode = m_midLinkNoCharge;
        break;
      case NoAuto:
        m_currentMode = m_noAuto;
        break;
      case PreloadDoubleShootFar:
        m_currentMode = m_preloadDoubleShootFar;
        break;
      case PreloadDoubleShootClose:
        m_currentMode = m_preloadDoubleShootClose;
        break;
    }
  }
}
