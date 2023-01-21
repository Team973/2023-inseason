package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;
import static frc.robot.shared.Constants.*;

import frc.robot.shared.Subsystem;

import edu.wpi.first.math.controller.PIDController;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

@Accessors(prefix = "m_")
public class Elevator implements Subsystem {

  private final TalonFX m_elevatorMotor;
  private final TalonFX m_elevatorFollowerMotor;

  private final DigitalInput m_bottomHall;

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

  private boolean m_isZeroed = false;

  public Elevator() {
    m_elevatorMotor = new TalonFX(ELEVATOR_FX_ID);
    m_elevatorFollowerMotor = new TalonFX(ELEVATOR_FOLLOWER_FX_ID);

    m_bottomHall = new DigitalInput(ELEVATOR_BOTTOM_HALL_SENSOR_ID);

    m_bottomHall = new DigitalInput(ELEVATOR_BOTTOM_HALL_SENSOR_ID);

    // Factory Default
    var motorConfig = new TalonFXConfiguration();
    // Motor Directions
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Neutral Mode
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Set motor to follow A
    m_elevatorMotor.getConfigurator().apply(motorConfig);
    m_elevatorFollowerMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_elevatorFollowerMotor.setControl(new Follower(ARM_FX_ID, true));

    // Motor feedback
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Ramp rate
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;

    // Velocity PID Parameters
    motorConfig.Slot0.kP = 0.0;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;
    motorConfig.Slot0.kS = 0.0;
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

  public void zeroSequence() {
    if (!m_isZeroed) {
      if (m_bottomHall.get()) {
        m_isZeroed = true;
        m_elevatorMotor.setRotorPosition(0.0);
      }
    }
  }

  public void update() {
    m_elevatorMotor.set(m_elevatorOutput);

    switch (m_elevatorState) {
      case Manual:
        break;
      case MotionMagic:
        double output = motionMagicOutput(m_offset, m_currentAngleInDegrees);
        m_elevatorMotor.set(output);
        break;
      case Idle:
        break;
    }
  }

  public void reset() {
    setElevatorOutput(0.0);
  }
}
