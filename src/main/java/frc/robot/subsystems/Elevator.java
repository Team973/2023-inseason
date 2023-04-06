package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.devices.GreyTalonFX;
import frc.robot.devices.GreyTalonFX.ControlMode;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.Subsystem;

import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Elevator implements Subsystem {

  private final GreyTalonFX m_elevatorMotor;
  private final GreyTalonFX m_elevatorFollowerMotor;

  private final DigitalInput m_bottomHall;
  private final DigitalInput m_topHall;

  private double m_elevatorOutput = 0.0;
  private double m_targetPosition = 0.0;

  private static final double GEAR_RATIO = (11.0 / 72.0);
  /** Pitch Diameter of sprocket in inches */
  private static final double SPROCKET_PD = 1.751;
  /** Circumference of sprocket in inches */
  private static final double SPROCKET_CIRCUMFERENCE = Math.PI * SPROCKET_PD;
  /** Degrees from floor */
  private static final double ANGLE = 51.519262;
  /** Sin of Elevator Angle. */
  private static final double SIN_OF_ANGLE = Math.sin(Math.toRadians(ANGLE));

  private static final double STOW_OFFSET = 7.628;

  private static final double MAX_HEIGHT = 27.58;

  @Getter @Setter private ElevatorState m_elevatorState = ElevatorState.Manual;
  @Getter private Preset m_preset = Preset.Stow;

  public enum ElevatorState {
    /** Manually control the motors with the joystick */
    Manual,
    /** Control the motors using position with Motion Magic. */
    ClosedLoop
  }

  public enum Preset {
    Floor(9.25),
    Hybrid(14.6),
    Mid(22.3),
    Hp(27.4),
    High(27.4),
    HighOffset(High.getValue() - 2.0),
    Stow(0.0),
    Manual(0.0),
    MiniHp(21.5);

    private final double value;

    Preset(double value) {
      this.value = value;
    }

    public double getValue() {
      return this.value;
    }
  }

  public Elevator() {
    m_elevatorMotor = new GreyTalonFX(ElevatorInfo.FX_ID, RobotInfo.CANIVORE_NAME);
    m_elevatorFollowerMotor = new GreyTalonFX(ElevatorInfo.FOLLOWER_FX_ID, RobotInfo.CANIVORE_NAME);
    m_bottomHall = new DigitalInput(ElevatorInfo.BOTTOM_HALL_SENSOR_ID);
    m_topHall = new DigitalInput(ElevatorInfo.TOP_HALL_SENSOR_ID);

    // Factory Default
    var motorConfig = m_elevatorMotor.getCurrentConfig();
    // Motor Directions
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 100.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Position PID Parameters
    motorConfig.Slot0.kP = 5.4;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;
    motorConfig.Slot0.kS = 0.0;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 55.0;
    motorConfig.MotionMagic.MotionMagicAcceleration = 180.0;

    // Set motor to follow A
    m_elevatorMotor.setConfig(motorConfig);
    m_elevatorFollowerMotor.setControl(new Follower(ElevatorInfo.FX_ID, false));

    m_elevatorMotor.setRotorPosition(0.0);
  }

  public void setTargetPosition(double target) {
    m_targetPosition = target;
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
    height = clamp(height, MAX_HEIGHT, STOW_OFFSET);
    m_targetPosition = getPositionFromHeight(height - STOW_OFFSET);
  }

  public void setPreset(Preset preset) {
    m_preset = preset;
    setHeight(preset.getValue());
  }

  private double getPositionFromHeight(double height) {
    return height / SIN_OF_ANGLE;
  }

  private double getHeightFromPosition(double position) {
    return position * SIN_OF_ANGLE;
  }

  private double clamp(double num, double max, double min) {
    return Math.min(max, Math.max(num, min));
  }

  public boolean getTopHall() {
    return !m_topHall.get();
  }

  public boolean getBottomHall() {
    return !m_bottomHall.get();
  }

  public boolean isAtHeight(double height) {
    return Math.abs(height - getHeight()) < 0.5;
  }

  public boolean isAtTargetHeight() {
    return isAtHeight(getHeightFromPosition(m_targetPosition));
  }

  public void dashboardUpdate() {}

  public void debugDashboardUpdate() {
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putNumber("Elevator Target Position", m_targetPosition);
    SmartDashboard.putNumber("Elevator Velocity", m_elevatorMotor.getRotorVelocity().getValue());
    SmartDashboard.putNumber("Elevator Height", getHeight());
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putBoolean("Elevator Bottom Hall", getBottomHall());
    SmartDashboard.putBoolean("Elevator Top Hall", getTopHall());
    SmartDashboard.putNumber(
        "Elevator Supply Current", m_elevatorMotor.getSupplyCurrent().getValue());
    SmartDashboard.putNumber(
        "Elevator Stator Current", m_elevatorMotor.getStatorCurrent().getValue());
  }

  public void update() {
    if (getTopHall()) {
      m_elevatorMotor.setRotorPosition(
          getPositionFromHeight(Elevator.MAX_HEIGHT - STOW_OFFSET)
              / SPROCKET_CIRCUMFERENCE
              / GEAR_RATIO);
    } else if (getBottomHall()) {
      m_elevatorMotor.setRotorPosition(0.0);
    }

    switch (m_elevatorState) {
      case Manual:
        if (getTopHall()) {
          m_elevatorOutput = clamp(m_elevatorOutput, 0.0, -1.0);
        } else if (getBottomHall()) {
          m_elevatorOutput = clamp(m_elevatorOutput, 1.0, 0.0);
        } else {
          m_elevatorOutput = clamp(m_elevatorOutput, 0.2, -0.2);
        }
        m_elevatorMotor.setControl(ControlMode.DutyCycleOut, m_elevatorOutput);

        m_targetPosition = getPosition();
        break;
      case ClosedLoop:
        double motorPosition = m_targetPosition / SPROCKET_CIRCUMFERENCE / GEAR_RATIO;
        m_elevatorMotor.setControl(ControlMode.MotionMagicVoltage, motorPosition);
        break;
      default:
        break;
    }
  }

  public void reset() {
    setElevatorOutput(0.0);
    m_elevatorMotor.setRotorPosition(0.0);
  }
}
