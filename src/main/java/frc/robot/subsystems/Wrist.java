package frc.robot.subsystems;

import frc.robot.devices.GreyPigeon;
import frc.robot.devices.GreyTalonFX;
import frc.robot.devices.GreyTalonFX.ControlMode;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.ClawInfo;
import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Superstructure.GamePiece;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Wrist implements Subsystem {
  private static final double STOW_OFFSET = 32.25 + 5.0; // +elevator slop
  private static final double WRIST_FF = 0.45;
  private static final double ENCODER_OFFSET = 304.189 - STOW_OFFSET;

  @Setter @Getter private WristState m_state = WristState.ClosedLoop;
  @Getter private WristPreset m_preset = WristPreset.Stow;

  private final GreyPigeon m_pigeon;
  private final CANcoder m_encoder;
  private final GreyTalonFX m_wristMotor;

  private final DigitalInput m_wristHall;

  @Getter private double m_targetAngle = STOW_OFFSET;

  @Setter private double m_motorOutput = 0.0;

  private final double ANGLE_TOLERANCE = 3.0; // degrees

  public enum WristState {
    Manual,
    ClosedLoop
  }

  public enum WristPreset {
    Floor(-107.59, -96.77),
    Hybrid(-44.6, -44.6),
    Mid(-118.22, -110.79),
    High(-111.09, -96.39),
    HighBack(-101.09, -74.39),
    Hp(-106.84, -95.36),
    Stow(STOW_OFFSET, STOW_OFFSET),
    ConeRight(-71.0, -74.0),
    MiniHp(-89.5, -86.0),
    PreScore(0.0, 0.0),
    Manual(0.0, 0.0),
    PreStow(STOW_OFFSET - 25, STOW_OFFSET - 25);

    private final double cube;
    private final double cone;

    WristPreset(double cube, double cone) {
      this.cube = cube;
      this.cone = cone;
    }

    public double getCubePreset() {
      return this.cube;
    }

    public double getConePreset() {
      return this.cone;
    }
  }

  public Wrist(GreyPigeon pigeon) {
    m_pigeon = pigeon;
    m_encoder = new CANcoder(ClawInfo.WRIST_ENCODER_ID, RobotInfo.CANIVORE_NAME);
    configEncoder();

    m_wristMotor = new GreyTalonFX(ClawInfo.WRIST_FX_ID, RobotInfo.CANIVORE_NAME);
    configWristMotor();

    m_wristHall = new DigitalInput(ClawInfo.WRIST_HALL_ID);
  }

  private void configWristMotor() {
    var motorConfig = m_wristMotor.getCurrentConfig();

    // Motor Directions
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Neutral Mode
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = 50;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor feedback
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = ClawInfo.WRIST_ENCODER_ID;

    // Ramp rate
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;

    // Position PID Parameters
    motorConfig.Slot0.kP = 43.0;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;
    motorConfig.Slot0.kS = 0.0;

    // Motion Magic
    motorConfig.MotionMagic.MotionMagicAcceleration = 12;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 30;
    m_wristMotor.setConfig(motorConfig);
  }

  private void configEncoder() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    m_encoder.getConfigurator().apply(encoderConfig);
  }

  public void setPreset(WristPreset nextPreset) {
    GamePiece currentGamePiece = Superstructure.getCurrentGamePiece();
    m_preset = nextPreset;
    // TODO handle cycle bug
    if (m_preset == WristPreset.Manual && m_state != WristState.ClosedLoop) {
      setTargetAngleDegrees(getCurrentAngleDegrees());

    } else if (m_preset == WristPreset.Manual && m_state == WristState.ClosedLoop) {
      // Intentionally left blank
    } else if (currentGamePiece == GamePiece.Cube) {
      setTargetAngleDegrees(m_preset.getCubePreset());
    } else {
      setTargetAngleDegrees(m_preset.getConePreset());
    }
  }

  public double getCurrentAngleDegrees() {
    return (m_encoder.getAbsolutePosition().getValue() * 360.0) - ENCODER_OFFSET;
  }

  private double getRawAngleDegrees() {
    return m_encoder.getAbsolutePosition().getValue() * 360.0;
  }

  public void setTargetAngleDegrees(double angle) {
    m_targetAngle = angle;
  }

  public double getVelocity() {
    return m_encoder.getVelocity().getValue() * 360.0;
  }

  public boolean getWristHall() {
    return !m_wristHall.get();
  }

  public boolean isAtTargetAngle() {
    return Math.abs(getCurrentAngleDegrees() - m_targetAngle) < ANGLE_TOLERANCE;
  }

  public void dashboardUpdate() {}

  public void debugDashboardUpdate() {
    SmartDashboard.putNumber("Wrist Angle", getCurrentAngleDegrees());
    SmartDashboard.putNumber("Wrist Angle Target", m_targetAngle);
    SmartDashboard.putString("Wrist Preset", m_preset.toString());
    SmartDashboard.putNumber("Wrist Stator", m_wristMotor.getStatorCurrent().getValue());
    SmartDashboard.putBoolean("Wrist Sensor", getWristHall());
    SmartDashboard.putNumber("Wrist Absolute Encoder", m_encoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("Wrist Raw Angle", getRawAngleDegrees());
    SmartDashboard.putNumber("Wrist Velocity", getVelocity());
  }

  public void update() {
    GamePiece currentGamePiece = Superstructure.getCurrentGamePiece();
    if (m_preset == WristPreset.Manual && m_state != WristState.ClosedLoop) {
      setTargetAngleDegrees(getCurrentAngleDegrees());
    } else if (m_preset == WristPreset.Manual && m_state == WristState.ClosedLoop) {
      // Intentionally left blank
    } else if (currentGamePiece == GamePiece.Cube) {
      setTargetAngleDegrees(m_preset.getCubePreset());
    } else {
      setTargetAngleDegrees(m_preset.getConePreset());
    }

    switch (m_state) {
      case Manual:
        setPreset(WristPreset.Manual);
        m_wristMotor.setControl(ControlMode.DutyCycleOut, m_motorOutput);
        break;
      case ClosedLoop:
        // If the original target angle is > 0, then don't gimbal. Make the max gimbaled angle 0
        var angle = m_targetAngle;
        if (m_targetAngle < 0) {
          angle = Math.min(0, m_targetAngle + m_pigeon.getPitch().getDegrees());
        }

        m_wristMotor.setControl(
            ControlMode.MotionMagicVoltage,
            (angle + ENCODER_OFFSET) / 360.0,
            true,
            Math.sin(Math.toRadians(getCurrentAngleDegrees())) * -WRIST_FF);
        break;
      default:
        break;
    }
  }

  public void reset() {
    setPreset(WristPreset.Stow);
  }
}
