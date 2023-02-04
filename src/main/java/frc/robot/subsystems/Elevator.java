package frc.robot.subsystems;

import static frc.robot.shared.Constants.*;
import static frc.robot.shared.RobotInfo.*;

import frc.robot.shared.RobotInfo;
import frc.robot.shared.Subsystem;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Elevator implements Subsystem {

  public static class Presets {
    /** Inches from floor */
    public static final double floor = 9.25;
    /** Inches from floor */
    public static final double mid = 34.5;
    /** Inches from floor */
    public static final double hp = 40.0;
    /** Inches from floor */
    public static final double high = 48.0;
  }

  private final TalonFX m_elevatorMotor;
  private final TalonFX m_elevatorFollowerMotor;

  private final DigitalInput m_bottomHall;
  private final DigitalInput m_topHall;

  private double m_elevatorOutput = 0.0;
  private double m_targetPosition = 0.0;
  private double m_manualSpeed = 0.0;

  private static final double GEAR_RATIO = (11.0 / 60.0);
  /** Pitch Diameter of sprocket in inches */
  private static final double SPROCKET_PD = 1.751;
  /** Circumference of sprocket in inches */
  private static final double SPROCKET_CIRCUMFERENCE = Math.PI * SPROCKET_PD;
  /** Degrees from floor */
  private static final double ANGLE = 51.519262;
  /** Sin of Elevator Angle. */
  public static final double SIN_OF_ANGLE = Math.sin(Math.toRadians(ANGLE));

  public static final double STOW_OFFSET = 7.628;

  @Getter @Setter private ElevatorState m_elevatorState;

  public enum ElevatorState {
    /** Manually control the motors with the joystick */
    Manual,
    /** Control the motors using position with Motion Magic. */
    ClosedLoop
  }

  public Elevator() {
    m_elevatorMotor = new TalonFX(ElevatorInfo.FX_ID, RobotInfo.CANIVORE_NAME);
    m_elevatorFollowerMotor = new TalonFX(ElevatorInfo.FOLLOWER_FX_ID, RobotInfo.CANIVORE_NAME);
    m_bottomHall = new DigitalInput(ElevatorInfo.BOTTOM_HALL_SENSOR_ID);
    m_topHall = new DigitalInput(ElevatorInfo.TOP_HALL_SENSOR_ID);

    // Factory Default
    var motorConfig = new TalonFXConfiguration();
    // Motor Directions
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Neutral Mode
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // Motor feedback
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Ramp rate
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;

    // Position PID Parameters
    motorConfig.Slot0.kP = 0.8;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;
    motorConfig.Slot0.kS = 0.0;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0;
    motorConfig.MotionMagic.MotionMagicAcceleration = 600.0;
    // motorConfig.MotionMagic.MotionMagicJerk = 50.0;

    // Set motor to follow A
    m_elevatorMotor.getConfigurator().apply(motorConfig);
    m_elevatorFollowerMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_elevatorFollowerMotor.setControl(new Follower(ElevatorInfo.FX_ID, false));

    m_elevatorMotor.setRotorPosition(0.0);
  }

  public void setElevatorOutput(double percent) {
    m_elevatorOutput = percent;
  }

  public double getHeight() {
    return getHeightFromPosition(getPosition()) + STOW_OFFSET;
  }

  public double getPosition() {
    return m_elevatorMotor.getRotorPosition().getValue() * SPROCKET_CIRCUMFERENCE * GEAR_RATIO;
  }

  public void setHeight(double height) {
    m_targetPosition = getPositionFromHeight(height - STOW_OFFSET);
  }

  private double getPositionFromHeight(double height) {
    return height / SIN_OF_ANGLE;
  }

  private double getHeightFromPosition(double position) {
    return position * SIN_OF_ANGLE;
  }

  public void setManualSpeed(double speed) {
    m_manualSpeed = speed;
    setElevatorState(ElevatorState.Manual);
  }

  public void setHpPreset() {
    setHeight(Presets.hp);
  }

  public void setHighPreset() {
    setHeight(Presets.high);
  }

  public void setMidPreset() {
    setHeight(Presets.mid);
  }

  public void setFloorPreset() {
    setHeight(Presets.floor);
  }

  public void update() {
    /*
    if ((m_elevatorOutput > 0.0 && m_topHall.get())
        || (m_elevatorOutput < 0.0 && m_bottomHall.get())) {
      m_elevatorOutput = 0.0;
    }
    */

    if (m_targetPosition > 23.0) {
      m_targetPosition = 23.0;
    }

    switch (m_elevatorState) {
      case Manual:
        m_elevatorMotor.set(m_manualSpeed);
        break;
      case ClosedLoop:
        m_elevatorMotor.setControl(
            new MotionMagicDutyCycle(m_targetPosition, false, 0.04, 0, true));
        break;
    }

    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator Target Position", m_targetPosition);
    SmartDashboard.putNumber("Elevator Velocity", m_elevatorMotor.getRotorVelocity().getValue());
  }

  public void reset() {
    setElevatorOutput(0.0);
  }
}
