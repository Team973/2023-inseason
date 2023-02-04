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
  @Setter @Getter private ClawState m_clawState;

  @Setter @Getter private GamePiece m_currentGamePiece;

  private double m_targetAngle = 0.0;
  private final TalonFX m_clawMotor;

  private double m_clawMotorOutput = 0.0;

  public enum GamePiece {
    Cube,
    Cone
  }

  public enum ClawState {
    In,
    Out,
    Neutral
  }

  public Claw() {
    m_clawMotor = new TalonFX(ClawInfo.FX_ID);
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
    m_clawMotor.getConfigurator().apply(motorConfig);
  }

  public void setClawMotorOutput(double percent) {
    m_clawMotorOutput = percent;
  }

  public void update() {
    m_clawMotor.set(m_clawMotorOutput);

    switch (m_clawState) {
      case In:
        switch (m_currentGamePiece) {
          case Cone:
            setClawMotorOutput(-0.5);
            break;
          case Cube:
            setClawMotorOutput(0.5);
            break;
        }
        break;
      case Out:
        switch (m_currentGamePiece) {
          case Cone:
            setClawMotorOutput(0.5);
            break;
          case Cube:
            setClawMotorOutput(-0.5);
            break;
        }
        break;
      case Neutral:
        setClawMotorOutput(0.0);
        break;
    }
  }

  public double getclawCurrentAngle() {
    double rot = m_clawMotor.getRotorPosition().getValue() * ClawInfo.GEAR_RATIO;
    return Rotation2d.fromRotations(rot).getDegrees();
  }

  public void setClawTargetAngle(double angle) {
    m_targetAngle = angle;
  }

  public void reset() {
    setClawMotorOutput(0.0);
  }
}
