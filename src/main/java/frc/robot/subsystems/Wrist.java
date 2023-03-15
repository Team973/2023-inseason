package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.shared.GreyTalonFX;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.ClawInfo;
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
public class Wrist implements Subsystem {

  private static final double STOW_OFFSET = 31.04;
  private static final double TENSED_STOW_OFFSET = 37.21;
  private static final double WRIST_FF = 0.4;

  public static class ConePresets {
    public static final double floor = -105.76;
    public static final double hybrid = -163.90;
    public static final double mid = -110.79;
    public static final double high = -94.39;
    public static final double highBack = high + 20.0;
    public static final double hp = -102.91;
    public static final double right = -74.0;
    public static final double stow = STOW_OFFSET;
    public static final double miniHp = -86.0;
    public static final double offset = STOW_OFFSET - 10;
  }

  public static class CubePresets {
    public static final double floor = -113;
    public static final double hybrid = -161.90;
    public static final double mid = -118.22;
    public static final double high = -111.09;
    public static final double highBack = high + 10.0;
    public static final double hp = -104.84;
    public static final double right = -71.0;
    public static final double stow = STOW_OFFSET;
    public static final double miniHp = -89.5;
    public static final double offset = STOW_OFFSET - 10;
  }

  @Setter @Getter private WristState m_state = WristState.Manual;
  @Getter private WristPreset m_preset = WristPreset.Stow;

  private final GreyTalonFX m_wristMotor;

  private final DigitalInput m_wristHall;

  private double m_targetAngle = STOW_OFFSET;

  @Setter private double m_motorOutput = 0.0;

  private final double ANGLE_TOLERANCE = 1.0; // degrees

  private final PositionVoltage m_wristPosition =
      new PositionVoltage(m_targetAngle / 360.0 / ClawInfo.GEAR_RATIO);

  public enum WristState {
    Manual,
    ClosedLoop
  }

  public enum WristPreset {
    Floor,
    Hybrid,
    Mid,
    High,
    HighBack,
    HP,
    Stow,
    ConeRight,
    MiniHp,
    Manual,
    Offset
  }

  public Wrist() {
    m_wristMotor = new GreyTalonFX(ClawInfo.WRIST_FX_ID, RobotInfo.CANIVORE_NAME);
    configWristMotor();

    m_wristHall = new DigitalInput(ClawInfo.WRIST_HALL_ID);

    m_wristMotor.setRotorPosition(STOW_OFFSET / ClawInfo.GEAR_RATIO / 360.0);
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

  public double getCurrentAngleDegrees() {
    double rot = m_wristMotor.getRotorPosition().getValue() * ClawInfo.GEAR_RATIO;
    return Rotation2d.fromRotations(rot).getDegrees();
  }

  private void setTargetAngleDegrees(double angle) {
    m_targetAngle = angle;
  }

  public double getVelocity() {
    return m_wristMotor.getVelocity().getValue() * ClawInfo.GEAR_RATIO * 360.0;
  }

  public boolean getWristHall() {
    return !m_wristHall.get();
  }

  public boolean isAtTargetAngle() {
    return Math.abs(getCurrentAngleDegrees() - m_targetAngle) < ANGLE_TOLERANCE;
  }

  public void setPreset(WristPreset preset) {
    m_preset = preset;
    GamePiece currentGamePiece = Robot.getCurrentGamePiece();
    switch (m_preset) {
      case Floor:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.floor);
        } else {
          setTargetAngleDegrees(ConePresets.floor);
        }
        break;
      case Mid:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.mid);
        } else {
          setTargetAngleDegrees(ConePresets.mid);
        }
        break;
      case High:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.high);
        } else {
          setTargetAngleDegrees(ConePresets.high);
        }
        break;
      case HighBack:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.highBack);
        } else {
          setTargetAngleDegrees(ConePresets.highBack);
        }
        break;
      case HP:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.hp);
        } else {
          setTargetAngleDegrees(ConePresets.hp);
        }
        break;
      case Stow:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.stow);
        } else {
          setTargetAngleDegrees(ConePresets.stow);
        }
        break;
      case ConeRight:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.stow);
        } else {
          setTargetAngleDegrees(ConePresets.right);
        }
        break;
      case MiniHp:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.miniHp);
        } else {
          setTargetAngleDegrees(ConePresets.miniHp);
        }
        break;
      case Offset:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.offset);
        } else {
          setTargetAngleDegrees(ConePresets.offset);
        }
        break;
      case Manual:
      default:
        break;
    }
  }

  @Override
  public void dashboardUpdate() {
    SmartDashboard.putNumber("Wrist Angle", getCurrentAngleDegrees());
    SmartDashboard.putNumber("Wrist Angle Target", m_targetAngle);
    SmartDashboard.putString("Wrist Preset", m_preset.toString());
    SmartDashboard.putNumber("Wrist Stator", m_wristMotor.getStatorCurrent().getValue());
    SmartDashboard.putBoolean("Wrist Sensor", getWristHall());
  }

  @Override
  public void update() {
    GamePiece currentGamePiece = Robot.getCurrentGamePiece();

    switch (m_preset) {
      case Floor:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.floor);
        } else {
          setTargetAngleDegrees(ConePresets.floor);
        }
        break;
      case Hybrid:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.hybrid);
        } else {
          setTargetAngleDegrees(ConePresets.hybrid);
        }
        break;
      case Mid:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.mid);
        } else {
          setTargetAngleDegrees(ConePresets.mid);
        }
        break;
      case High:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.high);
        } else {
          setTargetAngleDegrees(ConePresets.high);
        }
        break;
      case HighBack:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.highBack);
        } else {
          setTargetAngleDegrees(ConePresets.highBack);
        }
        break;
      case HP:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.hp);
        } else {
          setTargetAngleDegrees(ConePresets.hp);
        }
        break;
      case Stow:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.stow);
        } else {
          setTargetAngleDegrees(ConePresets.stow);
        }
        break;
      case ConeRight:
        if (currentGamePiece == GamePiece.Cube) {
          setTargetAngleDegrees(CubePresets.right);
        } else {
          setTargetAngleDegrees(ConePresets.right);
        }
        break;
      case Manual:
      default:
        break;
    }

    switch (m_state) {
      case Manual:
        m_wristMotor.set(m_motorOutput);
        setTargetAngleDegrees(getCurrentAngleDegrees());
        setPreset(WristPreset.Manual);
        break;
      case ClosedLoop:
        m_wristMotor.setControl(
            m_wristPosition
                .withPosition(m_targetAngle / 360.0 / ClawInfo.GEAR_RATIO)
                .withFeedForward(Math.sin(Math.toRadians(getCurrentAngleDegrees())) * -WRIST_FF));
        break;
      default:
        break;
    }
  }

  @Override
  public void reset() {
    setPreset(WristPreset.Stow);
    m_wristMotor.setRotorPosition(TENSED_STOW_OFFSET / 360.0 / ClawInfo.GEAR_RATIO);
  }
}
