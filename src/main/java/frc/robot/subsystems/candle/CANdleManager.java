package frc.robot.subsystems.candle;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.shared.Conversions;
import frc.robot.shared.CrashTracker;
import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Superstructure;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class CANdleManager implements Subsystem {
  private static final int NUM_LEDS = 8 + 17 + 14 + 17;

  public enum LightState {
    Cube,
    Cone,
    GotIt,
    Emergency,
    RainbowBarf,
    Off,
    Balance
  }

  @Setter @Getter private LightState m_lightState = LightState.Off;
  private final CANdle m_candle = new CANdle(CANdleInfo.ID, "rio");

  private boolean m_flashLEDsOn = false;
  private double m_flashStartTime = 0.0;

  private static final double FLASH_DELAY_MSEC = 250.0;
  private static final double GOTIT_DELAY_MSEC = 80.0;

  public CANdleManager() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1.0;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  public void setLightWithGamePiece() {
    switch (Superstructure.getCurrentGamePiece()) {
      case Cube:
        setLightState(LightState.Cube);
        break;
      case Cone:
        setLightState(LightState.Cone);
        break;
      case None:
      default:
        setLightState(LightState.Off);
        break;
    }
  }

  private void setAlternate(RGBColor color1, RGBColor color2, double timeOut) {
    if (!m_flashLEDsOn && Conversions.Time.getMsecTime() - m_flashStartTime >= timeOut) {
      setColor(color1);
      m_flashStartTime = Conversions.Time.getMsecTime();
      m_flashLEDsOn = true;
    } else if (Conversions.Time.getMsecTime() - m_flashStartTime >= timeOut) {
      setColor(color2);
      m_flashStartTime = Conversions.Time.getMsecTime();
      m_flashLEDsOn = false;
    }
  }

  private void setFlashing(RGBColor color, double timeOut) {
    setAlternate(color, CANdleColors.off, timeOut);
  }

  private void setColor(RGBColor color) {
    m_candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
  }

  public void dashboardUpdate() {}

  public void debugDashboardUpdate() {
    SmartDashboard.putString("Candle State", m_lightState.toString());
  }

  public void update() {
    if (CrashTracker.isExceptionHappened()) {
      m_lightState = LightState.Emergency;
    }

    switch (m_lightState) {
      case Cone:
        m_candle.setLEDs(255, 150, 0); // set the CANdle LEDs to yellow
        break;
      case Cube:
        m_candle.setLEDs(170, 0, 255); // set the CANdle LEDs to purple
        break;
      case Emergency:
        setFlashing(CANdleColors.emergency, FLASH_DELAY_MSEC);
        break;
      case GotIt:
        setFlashing(CANdleColors.gotIt, GOTIT_DELAY_MSEC);
        break;
      case RainbowBarf:
        m_candle.animate(new RainbowAnimation(1, 100.0, NUM_LEDS));
        break;
      case Balance:
        setColor(CANdleColors.balance);
        break;
      case Off:
        setColor(CANdleColors.off);
        break;
      default:
        break;
    }
  }

  public void reset() {
    m_lightState = LightState.Off;
  }
}
