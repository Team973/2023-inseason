package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

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
    Off
  }

  @Setter @Getter private LightState m_lightState = LightState.Off;
  private final CANdle m_candle = new CANdle(CANDLE_ID, "rio");

  public CANdleManager() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  public void update() {
    switch (m_lightState) {
      case Cone:
        break;
      case Cube:
        break;
      case Off:
        break;
      default:
        break;
    }
  }

  public void reset() {
    m_lightState = LightState.Off;
  }
}
