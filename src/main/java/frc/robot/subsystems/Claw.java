package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Claw implements Subsystem {

  public static class ConePresets {
    public static final double floor = -107.41;
    public static final double mid = -104.79;
    public static final double high = -92.97;
    public static final double hp = -102.94;
    public static final double stow = STOW_OFFSET;
    public static final double right = -63.0;
  }

  public static class CubePresets {
    public static final double floor = -115;
    public static final double mid = -120.24;
    public static final double high = -105.09;
    public static final double hp = -108.96;
    public static final double stow = STOW_OFFSET;
  }

  @Setter @Getter private IntakeState m_intakeState = IntakeState.Neutral;
  @Setter @Getter private WristState m_wristState = WristState.Manual;
  @Setter @Getter private GamePiece m_currentGamePiece;
  @Setter @Getter private WristPreset m_wristPreset = WristPreset.Stow;

  private final GreyTalonFX m_intakeMotor;
  private final GreyTalonFX m_wristMotor;
  private final DigitalInput m_wristHall;
  private static final double STOW_OFFSET = 31.04;

  private double m_targetAngle = STOW_OFFSET;
  private double m_intakeStator = 0.0;
  private double m_intakeMotorOutput = 0.0;
  @Setter private double m_wristMotorOutput = 0.0;
  private double m_statorCurrentLimit = 60.0;
  private double m_supplyCurrentLimit = 100.0;
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

  private static final double WRIST_FF = 0.4;

  public Claw() {
    m_intakeMotor = new GreyTalonFX(ClawInfo.INTAKE_FX_ID, RobotInfo.CANIVORE_NAME);
    m_wristMotor = new GreyTalonFX(ClawInfo.WRIST_FX_ID, RobotInfo.CANIVORE_NAME);
    m_wristHall = new DigitalInput(ClawInfo.WRIST_HALL_ID);
    configIntakeMotor();
    configWristMotor();

    m_wristMotor.setRotorPosition(STOW_OFFSET / ClawInfo.GEAR_RATIO / 360.0);
  }

  private void configIntakeMotor() {
    var motorConfig = new TalonFXConfiguration();

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit;
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

    // Position PID Parameters
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
    return Math.abs(m_intakeStator) > m_statorCurrentLimit - 10.0;
  }

  public void dashboardUpdate() {
    SmartDashboard.putNumber("Intake Stator", m_intakeStator);
    SmartDashboard.putNumber("Intake Supply", m_intakeMotor.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("Intake Velocity", m_intakeMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Claw Angle", getClawCurrentAngle());
    SmartDashboard.putNumber("Claw Angle Target", m_targetAngle);
    SmartDashboard.putBoolean("Game Piece", checkForGamePiece());
  }

  public boolean getWristHall() {
    return !m_wristHall.get();
  }

  public void update() {
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
            m_wristPosition
                .withPosition(m_targetAngle / 360.0 / ClawInfo.GEAR_RATIO)
                .withFeedForward(Math.sin(Math.toRadians(getClawCurrentAngle())) * -WRIST_FF));
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
    /*if (getWristHall()) {
      m_wristMotor.setRotorPosition(STOW_OFFSET);
    }*/
  }

  public void reset() {
    setIntakeState(IntakeState.Neutral);
  }

  public boolean isAtAngle() {
    return Math.abs(getClawCurrentAngle() - m_targetAngle) < ANGLE_TOLERANCE;
  }
}
