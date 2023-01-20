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
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator implements Subsystem {

  private final TalonFX m_elevatorMotor;
  private final TalonFX m_elevatorFollowerMotor;

  private final DigitalInput m_bottamHall;

  private double m_elevatorOutput = 0.0;

  private boolean m_isZeroed = false;

  public Elevator() {
    m_elevatorMotor = new TalonFX(ELEVATOR_FX_ID);
    m_elevatorFollowerMotor = new TalonFX(ELEVATOR_FOLLOWER_FX_ID);

    m_elevatorFollowerMotor.follow(m_elevatorMotor);

    m_bottamHall = new DigitalInput(ELEVATOR_BOTTOM_HALL_SENSOR_ID);

    final SupplyCurrentLimitConfiguration m_currentLimit =
        new SupplyCurrentLimitConfiguration(true, 40, 50, 0.05);
    final StatorCurrentLimitConfiguration m_statorLimit =
        new StatorCurrentLimitConfiguration(true, 80, 100, 0.05);

    // Factory Default
    m_elevatorMotor.configFactoryDefault();

    // Motor Directions
    m_elevatorMotor.setInverted(TalonFXInvertType.CounterClockwise);

    // Neutral Mode
    m_elevatorMotor.setNeutralMode(NeutralMode.Brake);

    // Current limits
    m_elevatorMotor.configSupplyCurrentLimit(m_currentLimit);
    m_elevatorMotor.configStatorCurrentLimit(m_statorLimit);

    // Deadband config
    m_elevatorMotor.configNeutralDeadband(0.01);

    // Set motor to follow A

    // Motor feedback
    m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    // Ramp rate
    m_elevatorMotor.configClosedloopRamp(0.0);

    // Voltage Compensation
    m_elevatorMotor.configVoltageCompSaturation(12.0);
    m_elevatorMotor.enableVoltageCompensation(true);

    // Velocity PID Parameters
    m_elevatorMotor.config_kP(0, 0.025, 30);
    m_elevatorMotor.config_kI(0, 0.0, 30);
    m_elevatorMotor.config_kD(0, 0.000, 30);
    m_elevatorMotor.config_kF(0, 0.048, 30);
  }

  public void setElevatorOutput(double percent) {
    m_elevatorOutput = percent;
  }

  public void zeroSequence() {
    if (!m_isZeroed) {
      if (m_bottamHall.get()) {
        m_isZeroed = true;
        m_elevatorMotor.setNeutralMode(NeutralMode.Brake);
        m_elevatorMotor.setSelectedSensorPosition(0.0);
      }
    }
  }

  public void update() {
    m_elevatorMotor.set(ControlMode.PercentOutput, m_elevatorOutput);
  }

  public void reset() {
    setElevatorOutput(0.0);
  }
}
