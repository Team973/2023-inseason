package frc.robot.subsystems.candle;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.Robot;
import frc.robot.shared.Conversions;
import frc.robot.shared.Subsystem;

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
    Emergency,
    AutoWaiting,
    PreloadWaiting,
    AutoSelected,
    RainbowBarf,
    Off
  }

  @Setter @Getter private LightState m_lightState = LightState.Off;
  private final CANdle m_candle = new CANdle(CANdleInfo.ID, "rio");

  private boolean m_flashLEDsOn = false;
  private double m_flashStartTime = 0.0;

  public CANdleManager() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1.0;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
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

  public void dashboardUpdate() {
    SmartDashboard.putString("Candle State", m_lightState.toString());
  }

  public void update() {
    if (Robot.isExceptionHappened()) {
      m_lightState = LightState.Emergency;
    }

    switch (m_lightState) {
      case Cone:
        setColor(CANdleColors.cone);
        break;
      case Cube:
        setColor(CANdleColors.cube);
        break;
      case Emergency:
        setFlashing(CANdleColors.emergency, 250.0);
        break;
      case AutoWaiting:
        setFlashing(CANdleColors.autoWaiting, 1250.0);
        break;
      case PreloadWaiting:
        setAlternate(CANdleColors.cone, CANdleColors.cube, 1000.0);
        break;
      case AutoSelected:
        setFlashing(CANdleColors.autoSelected, 1250.0);
        break;
      case RainbowBarf:
        m_candle.animate(new RainbowAnimation(1, 10.0, NUM_LEDS));
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
