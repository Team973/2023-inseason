package frc.robot.subsystems;

import static frc.robot.shared.Constants.*;
import static frc.robot.shared.RobotInfo.*;

import frc.robot.shared.Subsystem;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Elevator implements Subsystem {

  private final TalonFX m_elevatorMotor;
  private final TalonFX m_elevatorFollowerMotor;

  private final DigitalInput m_bottomHall;

  private double m_elevatorOutput = 0.0;
  private double m_targetPosition = 0.0;
  private double m_targetAngleInDegrees = 0.0;

  private static final double ELEVATOR_GEAR_RATIO = (12.0 / 60.0);
  /** Pitch Diameter of sprocket in inches */
  private static final double ELEVATOR_SPROCKET_PD = 1.751;
  /** Circumference of sprocket in inches */
  private static final double ELEVATOR_SPROCKET_CIRCUMFERENCE = Math.PI * ELEVATOR_SPROCKET_PD;
  /** Degrees from floor */
  private static final double ELEVATOR_ANGLE = 51.519262;
  /** Sin of Elevator Angle. */
  public static final double SIN_OF_ELEVATOR_ANGLE = Math.sin(ELEVATOR_ANGLE);

  @Getter
  @Setter
  private ElevatorState m_elevatorState;

  public enum ElevatorState {
    /** Manually control the motors with the joystick */
    Manual,
    /** Control the motors using position with Motion Magic. */
    ClosedLoop
  }

  private boolean m_isZeroed = false;

  public Elevator() {
    m_elevatorMotor = new TalonFX(ELEVATOR_FX_ID);
    m_elevatorFollowerMotor = new TalonFX(ELEVATOR_FOLLOWER_FX_ID);
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

    m_elevatorMotor.getConfigurator().apply(motorConfig);
    m_elevatorFollowerMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_elevatorFollowerMotor.setControl(new Follower(ARM_FX_ID, true));
  }

  public void setElevatorOutput(double percent) {
    m_elevatorOutput = percent;
  }

  public double getHeight() {
    return getHeightFromPosition(getPosition());
  }

  private double getPosition() {
    return m_elevatorMotor.getRotorPosition().getValue()
        * ELEVATOR_SPROCKET_CIRCUMFERENCE
        * ELEVATOR_GEAR_RATIO;
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
        m_elevatorMotor.set(m_targetAngleInDegrees);
        break;
      case ClosedLoop:
        MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(m_targetPosition);
        m_elevatorMotor.setControl(motionMagic);
        break;
    }
  }

  public void reset() {
    setElevatorOutput(0.0);
  }
}
