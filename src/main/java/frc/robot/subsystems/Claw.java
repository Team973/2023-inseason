package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.Robot;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.shared.GreyTalonFX;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.Subsystem;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Claw implements Subsystem {

  public static class ConePresets {
    public static final double floor = -138.45;
    public static final double mid = -135.83;
    public static final double high = -124.01;
    public static final double hp = -133.98;
    public static final double stow = 0.0;
  }

  public static class CubePresets {
    public static final double floor = -146.04;
    public static final double mid = -151.28;
    public static final double high = -136.13;
    public static final double hp = -140.00;
    public static final double stow = 0.0;
  }

  @Setter @Getter private IntakeState m_intakeState = IntakeState.Neutral;
  @Setter @Getter private WristState m_wristState = WristState.Manual;
  private GamePiece m_currentGamePiece = Robot.getCurrentGamePiece();
  @Setter @Getter private WristPreset m_wristPreset = WristPreset.Stow;

  private final GreyTalonFX m_intakeMotor;
  private final GreyTalonFX m_wristMotor;

  private double m_targetAngle = 0.0;
  private double m_intakeStator = 0.0;
  private double m_intakeMotorOutput = 0.0;
  @Setter private double m_wristMotorOutput = 0.0;
  private double m_statorCurrentLimit = 60.0;
  private final double ANGLE_TOLERANCE = 1.0; // degrees

  private final PositionVoltage m_wristPosition =
      new PositionVoltage(m_targetAngle / 360.0 / ClawInfo.GEAR_RATIO);

  public enum IntakeState {
    In,
    Out,
    Hold,
    Neutral
  }

  public enum WristState {
    Manual,
    ClosedLoop
  }

  public enum WristPreset {
    Floor,
    Mid,
    High,
    HP,
    Stow,
    Manual
  }

  public Claw() {
    m_intakeMotor = new GreyTalonFX(ClawInfo.INTAKE_FX_ID, RobotInfo.CANIVORE_NAME);
    m_wristMotor = new GreyTalonFX(ClawInfo.WRIST_FX_ID, RobotInfo.CANIVORE_NAME);
    configIntakeMotor();
    configWristMotor();

    m_wristMotor.setRotorPosition(0.0);
  }

  private void configIntakeMotor() {
    var motorConfig = new TalonFXConfiguration();

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = 100;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    m_intakeMotor.getConfigurator().apply(motorConfig);
  }

  private void configWristMotor() {
    var motorConfig = new TalonFXConfiguration();

    // Motor Directions
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Neutral Mode
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor feedback
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Ramp rate
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;

    // Velocity PID Parameters
    motorConfig.Slot0.kP = 2.0;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;
    motorConfig.Slot0.kS = 0.0;

    m_wristMotor.getConfigurator().apply(motorConfig);
  }

  public double getClawCurrentAngle() {
    double rot = m_wristMotor.getRotorPosition().getValue() * ClawInfo.GEAR_RATIO;
    return Rotation2d.fromRotations(rot).getDegrees();
  }

  public void setWristTargetAngle(double angle) {
    m_targetAngle = angle;
  }

  public boolean checkForGamePiece() {
    return m_intakeStator < m_statorCurrentLimit;
  }

  public void dashboardUpdate() {
    SmartDashboard.putNumber("Intake Stator", m_intakeStator);
    SmartDashboard.putNumber("Intake Supply", m_intakeMotor.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("Intake Velocity", m_intakeMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Claw Angle", getClawCurrentAngle());
  }

  public void update() {
    m_currentGamePiece = Robot.getCurrentGamePiece();
    m_intakeStator = m_intakeMotor.getStatorCurrent().getValue();

    switch (m_intakeState) {
      case In:
        m_intakeMotorOutput = 0.5;
        break;
      case Out:
        m_intakeMotorOutput = -0.5;
        break;
      case Hold:
        m_intakeMotorOutput = 0.1;
        break;
      case Neutral:
      default:
        m_intakeMotorOutput = 0.0;
        break;
    }

    if (m_currentGamePiece == GamePiece.Cube) {
      m_intakeMotorOutput *= -1.0;
    }

    m_intakeMotor.set(m_intakeMotorOutput);

    switch (m_wristState) {
      case Manual:
        m_wristMotor.set(m_wristMotorOutput);
        setWristTargetAngle(getClawCurrentAngle());
        setWristPreset(WristPreset.Manual);
        break;
      case ClosedLoop:
        m_wristMotor.setControl(
            m_wristPosition.withPosition(m_targetAngle / 360.0 / ClawInfo.GEAR_RATIO));
        break;
      default:
        break;
    }

    switch (m_wristPreset) {
      case Floor:
        if (m_currentGamePiece == GamePiece.Cube) {
          setWristTargetAngle(CubePresets.floor);
        } else {
          setWristTargetAngle(ConePresets.floor);
        }
        break;
      case Mid:
        if (m_currentGamePiece == GamePiece.Cube) {
          setWristTargetAngle(CubePresets.mid);
        } else {
          setWristTargetAngle(ConePresets.mid);
        }
        break;
      case High:
        if (m_currentGamePiece == GamePiece.Cube) {
          setWristTargetAngle(CubePresets.high);
        } else {
          setWristTargetAngle(ConePresets.high);
        }
        break;
      case HP:
        if (m_currentGamePiece == GamePiece.Cube) {
          setWristTargetAngle(CubePresets.hp);
        } else {
          setWristTargetAngle(ConePresets.hp);
        }
        break;
      case Stow:
        if (m_currentGamePiece == GamePiece.Cube) {
          setWristTargetAngle(CubePresets.stow);
        } else {
          setWristTargetAngle(ConePresets.stow);
        }
        break;
      case Manual:
      default:
        break;
    }
  }

  public void reset() {
    setIntakeState(IntakeState.Neutral);
  }

  public boolean isAtAngle() {
    return Math.abs(getClawCurrentAngle() - m_targetAngle) < ANGLE_TOLERANCE;
  }
}
