package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.shared.Subsystem;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Claw implements Subsystem {
  @Setter @Getter private IntakeState m_intakeState;

  @Setter @Getter private GamePiece m_currentGamePiece;

  private double m_targetAngle = 0.0;
  private final TalonFX m_intakeMotor;

  private double m_intakeMotorOutput = 0.0;

  public enum GamePiece {
    Cube,
    Cone
  }

  public enum IntakeState {
    In,
    Out,
    Neutral
  }

  public Claw() {
    m_intakeMotor = new TalonFX(ClawInfo.INTAKE_FX_ID);
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
    m_intakeMotor.getConfigurator().apply(motorConfig);
  }

  public void setIntakeMotorOutput(double percent) {
    m_intakeMotorOutput = percent;
  }

  public void update() {
    m_intakeMotor.set(m_intakeMotorOutput);

    switch (m_intakeState) {
      case In:
        switch (m_currentGamePiece) {
          case Cone:
            setIntakeMotorOutput(-0.5);
            break;
          case Cube:
            setIntakeMotorOutput(0.5);
            break;
        }
        break;
      case Out:
        switch (m_currentGamePiece) {
          case Cone:
            setIntakeMotorOutput(0.5);
            break;
          case Cube:
            setIntakeMotorOutput(-0.5);
            break;
        }
        break;
      case Neutral:
        setIntakeMotorOutput(0.0);
        break;
    }
  }

  public double getclawCurrentAngle() {
    double rot = m_intakeMotor.getRotorPosition().getValue() * ClawInfo.GEAR_RATIO;
    return Rotation2d.fromRotations(rot).getDegrees();
  }

  public void setClawTargetAngle(double angle) {
    m_targetAngle = angle;
  }

  public void reset() {
    setIntakeMotorOutput(0.0);
  }
}
