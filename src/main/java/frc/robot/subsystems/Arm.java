package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.shared.Subsystem;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Arm implements Subsystem {

  private double m_targetAngle = 0.0;
  private final TalonFX m_wristMotor;

  private final Solenoid m_armSolenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, ARM_SOLENOID_ID);

  private ExtensionState m_extensionState = ExtensionState.RETRACTED;

  private double m_wristMotorOutput = 0.0;

  private enum ExtensionState {
    RETRACTED,
    EXTENDED
  }

  public Arm() {
    m_wristMotor = new TalonFX(ARM_FX_ID);
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
    m_wristMotor.getConfigurator().apply(motorConfig);
  }

  public void setWristMotorOutput(double percent) {
    m_wristMotorOutput = percent;
  }

  public void update() {
    m_wristMotor.set(m_wristMotorOutput);

    switch (m_extensionState) {
      case RETRACTED:
        m_armSolenoid.set(false);
        break;
      case EXTENDED:
        m_armSolenoid.set(true);
        break;
      default:
        break;
    }
  }

  public double getWristCurrentAngle() {
    double rot = m_wristMotor.getRotorPosition().getValue() * wristGearRatio();
    return Rotation2d.fromRotations(rot).getDegrees();
  }

  public void setWristTargetAngle(double angle) {
    m_targetAngle = angle;
  }

  public void reset() {
    setWristMotorOutput(0.0);
  }
}
