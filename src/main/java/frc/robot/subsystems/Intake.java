package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.shared.Subsystem;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Intake implements Subsystem {
  @Setter private IntakeState m_intakeState;
  @Setter @Getter private GamePiece m_currentGamePiece;
  private final TalonFX m_intakeMotor;

  public enum GamePiece {
    Cube,
    Cone
  }

  public enum IntakeState {
    In,
    Out,
    Neutral
  }

  public Intake() {
    m_intakeState = IntakeState.Neutral;
    m_intakeMotor = new TalonFX(INTAKE_FX_ID);

    // Factory Default
    var motorConfig = new TalonFXConfiguration();
    // Motor Directions
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Neutral Mode
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Set motor to follow A
    m_intakeMotor.getConfigurator().apply(motorConfig);

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

  public void intake() {
    m_intakeState = IntakeState.In;
  }

  public void outtake() {
    m_intakeState = IntakeState.Out;
  }

  public void stopIntake() {
    m_intakeState = IntakeState.Neutral;
  }

  public void update() {
    switch (m_intakeState) {
      case In:
        m_intakeMotor.set(m_currentGamePiece == GamePiece.Cube ? 1.0 : -1.0);
        break;
      case Out:
        m_intakeMotor.set(m_currentGamePiece == GamePiece.Cube ? -1.0 : 1.0);
        break;
      case Neutral:
        m_intakeMotor.set(0.0);
        break;
      default:
        break;
    }
  }

  public void reset() {
    m_intakeState = IntakeState.Neutral;
  }
}
