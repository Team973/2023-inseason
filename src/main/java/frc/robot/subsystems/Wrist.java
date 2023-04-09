package frc.robot.subsystems;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Wrist implements Subsystem {

  private static final Rotation2d STOW_OFFSET = Rotation2d.fromDegrees(32.25);
  private static final double WRIST_FF = 0.45;
  private static final Rotation2d ENCODER_OFFSET =
      Rotation2d.fromDegrees(304.189).minus(STOW_OFFSET);

  @Setter @Getter private WristState m_state = WristState.Manual;
  @Getter private WristPreset m_preset = WristPreset.Stow;

  private final CANcoder m_encoder;
  private final GreyTalonFX m_wristMotor;

  private final DigitalInput m_wristHall;

  @Getter @Setter private Rotation2d m_targetAngle = STOW_OFFSET;

  @Setter private double m_motorOutput = 0.0;

  private final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.0);

  public enum WristState {
    Manual,
    ClosedLoop
  }

  @Accessors(prefix = "m_")
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
    PreStow(
        STOW_OFFSET.minus(Rotation2d.fromDegrees(25)),
        STOW_OFFSET.minus(Rotation2d.fromDegrees(25)));

    @Getter private final Rotation2d m_cubePreset;
    @Getter private final Rotation2d m_conePreset;

    WristPreset(Rotation2d cube, Rotation2d cone) {
      m_cubePreset = cube;
      m_conePreset = cone;
    }

    WristPreset(double cubeDegrees, double coneDegrees) {
      m_cubePreset = Rotation2d.fromDegrees(cubeDegrees);
      m_conePreset = Rotation2d.fromDegrees(coneDegrees);
    }
  }

  public Wrist() {
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
    motorConfig.Slot0.kP = 50.0; // 84.0;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;
    motorConfig.Slot0.kS = 0.0;

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
      setTargetAngle(getCurrentAngle());

    } else if (m_preset == WristPreset.Manual && m_state == WristState.ClosedLoop) {
      // Intentionally left blank
    } else if (currentGamePiece == GamePiece.Cube) {

      setTargetAngle(m_preset.getCubePreset());
    } else {
      setTargetAngle(m_preset.getConePreset());
    }
  }

  public Rotation2d getCurrentAngle() {
    return getRawAngle().minus(ENCODER_OFFSET);
  }

  private Rotation2d getRawAngle() {
    return Rotation2d.fromRotations(m_encoder.getAbsolutePosition().getValue());
  }

  public Rotation2d getVelocity() {
    return Rotation2d.fromRotations(m_encoder.getVelocity().getValue());
  }

  public boolean getWristHall() {
    return !m_wristHall.get();
  }

  public boolean isAtTargetAngle() {
    return Math.abs(getCurrentAngle().minus(m_targetAngle).getDegrees())
        < ANGLE_TOLERANCE.getDegrees();
  }

  public void dashboardUpdate() {}

  public void debugDashboardUpdate() {
    SmartDashboard.putNumber("Wrist Angle", getCurrentAngle().getDegrees());
    SmartDashboard.putNumber("Wrist Angle Target", m_targetAngle.getDegrees());
    SmartDashboard.putString("Wrist Preset", m_preset.toString());
    SmartDashboard.putNumber("Wrist Stator", m_wristMotor.getStatorCurrent().getValue());
    SmartDashboard.putBoolean("Wrist Sensor", getWristHall());
    SmartDashboard.putNumber("Wrist Absolute Encoder", m_encoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("Wrist Raw Angle", getRawAngle().getDegrees());
    SmartDashboard.putNumber("Wrist Velocity", getVelocity().getDegrees());
  }

  public void update() {
    GamePiece currentGamePiece = Superstructure.getCurrentGamePiece();
    if (m_preset == WristPreset.Manual && m_state != WristState.ClosedLoop) {
      setTargetAngle(getCurrentAngle());
    } else if (m_preset == WristPreset.Manual && m_state == WristState.ClosedLoop) {
      // Intentionally left blank
    } else if (currentGamePiece == GamePiece.Cube) {
      setTargetAngle(m_preset.getCubePreset());
    } else {
      setTargetAngle(m_preset.getConePreset());
    }

    switch (m_state) {
      case Manual:
        setPreset(WristPreset.Manual);
        m_wristMotor.setControl(ControlMode.DutyCycleOut, m_motorOutput);
        break;
      case ClosedLoop:
        m_wristMotor.setControl(
            ControlMode.PositionVoltage,
            m_targetAngle.plus(ENCODER_OFFSET).getRotations(),
            true,
            getCurrentAngle().getSin() * -WRIST_FF);
        break;
      default:
        break;
    }
  }

  public void reset() {
    setPreset(WristPreset.Stow);
  }
}
