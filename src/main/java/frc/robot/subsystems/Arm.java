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

public class Arm implements Subsystem {

  private final TalonFX m_armMotor;

  private double m_armMotorOutput = 0.0;

  public Arm() {
    m_armMotor = new TalonFX(ARM_FX_ID);

    final SupplyCurrentLimitConfiguration m_currentLimit =
        new SupplyCurrentLimitConfiguration(true, 40, 50, 0.05);
    final StatorCurrentLimitConfiguration m_statorLimit =
        new StatorCurrentLimitConfiguration(true, 80, 100, 0.05);

    // Factory Default
    m_armMotor.configFactoryDefault();

    // Motor Directions
    m_armMotor.setInverted(TalonFXInvertType.CounterClockwise);

    // Neutral Mode
    m_armMotor.setNeutralMode(NeutralMode.Brake);

    // Current limits
    m_armMotor.configSupplyCurrentLimit(m_currentLimit);
    m_armMotor.configStatorCurrentLimit(m_statorLimit);

    // Deadband config
    m_armMotor.configNeutralDeadband(0.01);

    // Set motor to follow A

    // Motor feedback
    m_armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    // Ramp rate
    m_armMotor.configClosedloopRamp(0.0);

    // Voltage Compensation
    m_armMotor.configVoltageCompSaturation(12.0);
    m_armMotor.enableVoltageCompensation(true);

    // Velocity PID Parameters
    m_armMotor.config_kP(0, 0.025, 30);
    m_armMotor.config_kI(0, 0.0, 30);
    m_armMotor.config_kD(0, 0.000, 30);
    m_armMotor.config_kF(0, 0.048, 30);
  }

  public void setArmMotorOutput(double percent) {
    m_armMotorOutput = percent;
  }

  public void update() {
    m_armMotor.set(ControlMode.PercentOutput, m_armMotorOutput);
  }

  public void reset() {
    setArmMotorOutput(0.0);
  }
}
