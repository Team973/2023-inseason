package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.shared.Constants.GamePiece;
import frc.robot.shared.Conversions;
import frc.robot.shared.Subsystem;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class CANdleManager implements Subsystem {
  public enum LightState {
    Cube,
    Cone,
    Emergency,
    AutoSelected,
    Off
  }

  @Setter @Getter private LightState m_lightState = LightState.Off;
  private final CANdle m_candle = new CANdle(CANdleInfo.ID, "rio");

  private boolean m_flashLEDsOn = false;
  private double m_flashStartTime = 0.0;

  private static final double FLASH_DELAY_TIME = 250.0;

  public CANdleManager() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1.0;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  public void setLightWithGamePiece(GamePiece gamePiece) {
    switch (gamePiece) {
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

  public void dashboardUpdate() {}

  public void update() {
    switch (m_lightState) {
      case Cone:
        m_candle.setLEDs(255, 150, 0); // set the CANdle LEDs to yellow
        break;
      case Cube:
        m_candle.setLEDs(170, 0, 255); // set the CANdle LEDs to purple
        break;
      case Emergency:
        if (!m_flashLEDsOn
            && Conversions.Time.getMsecTime() - m_flashStartTime >= FLASH_DELAY_TIME) {
          m_candle.setLEDs(255, 0, 0); // set the CANdle LEDs to red
          m_flashStartTime = Conversions.Time.getMsecTime();
          m_flashLEDsOn = true;
        } else if (Conversions.Time.getMsecTime() - m_flashStartTime >= FLASH_DELAY_TIME) {
          m_candle.setLEDs(0, 0, 0);
          m_flashStartTime = Conversions.Time.getMsecTime();
          m_flashLEDsOn = false;
        }
        break;
      case AutoSelected:
        if (!m_flashLEDsOn
            && Conversions.Time.getMsecTime() - m_flashStartTime >= FLASH_DELAY_TIME) {
          m_candle.setLEDs(0, 0, 255); // set the CANdle LEDs to red
          m_flashStartTime = Conversions.Time.getMsecTime();
          m_flashLEDsOn = true;
        } else if (Conversions.Time.getMsecTime() - m_flashStartTime >= FLASH_DELAY_TIME) {
          m_candle.setLEDs(0, 0, 0);
          m_flashStartTime = Conversions.Time.getMsecTime();
          m_flashLEDsOn = false;
        }
      case Off:
        m_candle.setLEDs(0, 0, 0); // set the CANdle LEDs to be offs
        break;
      default:
        break;
    }
  }

  public void reset() {
    m_lightState = LightState.Off;
  }
}
