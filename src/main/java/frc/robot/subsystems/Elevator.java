package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;
import static frc.robot.shared.Constants.*;

import frc.robot.shared.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Elevator implements Subsystem {

  private final TalonFX m_elevatorMotor;
  private final TalonFX m_elevatorFollowerMotor;

  private double m_elevatorOutput = 0.0;
  private double m_offset = 0.0;
  private double m_angularRate = 0.0;
  private double m_translationalValue = 0.0;
  private double m_currentAngleInDegrees = 0.0;
  private double m_elevatorPosition = 0.0;
  private double m_targetPosition = 0.0;

  @Getter
  @Setter
  private ElevatorState m_elevatorState = ElevatorState.Idle;
  @Getter
  @Setter
  private ElevatorPos m_elevatorPos;

  private PIDController m_elevatorPID = new PIDController(0.02, 0.0, 0.0);

  public enum ElevatorState {
    /** Control the motors using position with Motion Magic. */
    MotionMagic,
    /** Staying in place after pressing a button. */
    Idle,
    /** Manually control the motors with the joystick */
    Manual
  }

  public enum ElevatorPos {
    Top,
    Middle,
    Bottom
  }

  public Elevator() {
    m_elevatorMotor = new TalonFX(ELEVATOR_FX_ID);
    m_elevatorFollowerMotor = new TalonFX(ELEVATOR_FOLLOWER_FX_ID);

    m_elevatorFollowerMotor.follow(m_elevatorMotor);

    final SupplyCurrentLimitConfiguration m_currentLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 0.05);
    final StatorCurrentLimitConfiguration m_statorLimit = new StatorCurrentLimitConfiguration(true, 80, 100, 0.05);

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

  public double motionMagicOutput(double offset, double currentAngleInDegrees) {
    m_elevatorPID.setSetpoint(0.0);
    double targetAngle = currentAngleInDegrees + offset;

    return m_elevatorPID.calculate(offset) - m_angularRate;
  }

  public double getHeight() {
    // TODO: update this after converting to pheonix pro
    return 0.0;
  }

  public double getPosition() {
    // TODO: update this after converting to pheonix pro
    return 0.0;
  }

  public void setHeight(double height) {
    m_targetPosition = getPositionFromHeight(height);
  }

  private double getPositionFromHeight(double height) {
    return height / Math.sin(ELEVATOR_ANGLE);
  }

  private double getHeightFromPosition(double position) {
    return position * Math.sin(ELEVATOR_ANGLE);
  }

  public void update() {
    m_elevatorMotor.set(ControlMode.PercentOutput, m_elevatorOutput);

    switch (m_elevatorState) {
      case Manual:
        break;
      case MotionMagic:
        double output = motionMagicOutput(m_offset, m_currentAngleInDegrees);
        m_elevatorMotor.set(TalonFXControlMode.PercentOutput, output);
        break;
      case Idle:
        break;
    }
  }

  public void reset() {
    setElevatorOutput(0.0);
  }
}
