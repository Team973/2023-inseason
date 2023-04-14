package frc.robot;

import java.util.Arrays;
import java.util.List;

import frc.robot.auto.modes.Bump1HoldCharge;
import frc.robot.auto.modes.Center1Charge;
import frc.robot.auto.modes.Flat1Charge;
import frc.robot.auto.modes.Flat1HoldCharge;
import frc.robot.auto.modes.Flat2Charge;
import frc.robot.auto.modes.Flat3;
import frc.robot.auto.modes.NoAuto;
import frc.robot.auto.modes.Test;
import frc.robot.shared.AutoCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;

public class AutoManager {
  private AutoCommand m_currentMode;
  private final List<AutoMode> m_availableAutoModes =
      Arrays.asList(
          AutoMode.Flat2Charge,
          AutoMode.Flat3,
          AutoMode.Flat1HoldCharge,
          AutoMode.Bump1HoldCharge,
          AutoMode.Test,
          AutoMode.Flat1Charge,
          AutoMode.Center1Charge,
          AutoMode.NoAuto);
  private int m_selectedMode = 0;

  public enum AutoMode {
    Test,
    Flat1Charge,
    Flat1HoldCharge,
    Bump1HoldCharge,
    Center1Charge,
    Flat2Charge,
    Flat3,
    NoAuto,
  }

  private final AutoCommand m_test;
  private final AutoCommand m_flat1Charge;
  private final AutoCommand m_flat1HoldCharge;
  private final AutoCommand m_bump1HoldCharge;
  private final AutoCommand m_center1Charge;
  private final AutoCommand m_noAuto;
  private final AutoCommand m_flat2Charge;
  private final AutoCommand m_flat3;

  public AutoManager(Drive drive, Superstructure superstructure) {
    m_test = new Test(superstructure);
    m_flat1Charge = new Flat1Charge(drive, superstructure);
    m_flat1HoldCharge = new Flat1HoldCharge(drive, superstructure);
    m_bump1HoldCharge = new Bump1HoldCharge(drive, superstructure);
    m_center1Charge = new Center1Charge(drive, superstructure);
    m_flat2Charge = new Flat2Charge(drive, superstructure);
    m_flat3 = new Flat3(drive, superstructure);
    m_noAuto = new NoAuto();
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
      case Flat1Charge:
        m_currentMode = m_flat1Charge;
        break;
      case Flat1HoldCharge:
        m_currentMode = m_flat1HoldCharge;
        break;
      case Bump1HoldCharge:
        m_currentMode = m_bump1HoldCharge;
        break;
      case Center1Charge:
        m_currentMode = m_center1Charge;
        break;
      case Flat2Charge:
        m_currentMode = m_flat2Charge;
        break;
      case Flat3:
        m_currentMode = m_flat3;
        break;
      case NoAuto:
        m_currentMode = m_noAuto;
        break;
    }
  }
}
