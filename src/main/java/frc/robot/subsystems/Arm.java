package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.shared.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Arm implements Subsystem {

  private final TalonFX m_wristMotor;

  private final Solenoid m_armSolenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, ARM_SOLENOID_ID);

  private ExtensionState m_extensionState = ExtensionState.RETRACTED;

  private double m_wristMotorOutput = 0.0;

  private enum ExtensionState {
    RETRACTED,
    EXTENDED
  }

  public Arm() {
    m_wristMotor = new TalonFX(ARM_FX_ID);

    final SupplyCurrentLimitConfiguration m_currentLimit =
        new SupplyCurrentLimitConfiguration(true, 40, 50, 0.05);
    final StatorCurrentLimitConfiguration m_statorLimit =
        new StatorCurrentLimitConfiguration(true, 80, 100, 0.05);

    // Factory Default
    m_wristMotor.configFactoryDefault();

    // Motor Directions
    m_wristMotor.setInverted(TalonFXInvertType.CounterClockwise);

    // Neutral Mode
    m_wristMotor.setNeutralMode(NeutralMode.Brake);

    // Current limits
    m_wristMotor.configSupplyCurrentLimit(m_currentLimit);
    m_wristMotor.configStatorCurrentLimit(m_statorLimit);

    // Deadband config
    m_wristMotor.configNeutralDeadband(0.01);

    // Set motor to follow A

    // Motor feedback
    m_wristMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    // Ramp rate
    m_wristMotor.configClosedloopRamp(0.0);

    // Voltage Compensation
    m_wristMotor.configVoltageCompSaturation(12.0);
    m_wristMotor.enableVoltageCompensation(true);

    // Velocity PID Parameters
    m_wristMotor.config_kP(0, 0.025, 30);
    m_wristMotor.config_kI(0, 0.0, 30);
    m_wristMotor.config_kD(0, 0.000, 30);
    m_wristMotor.config_kF(0, 0.048, 30);
  }

  public void setWristMotorOutput(double percent) {
    m_wristMotorOutput = percent;
  }

  public void update() {
    m_wristMotor.set(ControlMode.PercentOutput, m_wristMotorOutput);

    switch (m_extensionState) {
      case RETRACTED:
        m_armSolenoid.set(false);
        break;
      case EXTENDED:
        m_armSolenoid.set(true);
        break;
      default:
        break;
    }
  }

  public void reset() {
    setWristMotorOutput(0.0);
  }
}
