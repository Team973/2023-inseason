package frc.robot.shared;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.ForwardLimitSourceValue;
import com.ctre.phoenixpro.signals.ForwardLimitTypeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.ReverseLimitSourceValue;
import com.ctre.phoenixpro.signals.ReverseLimitTypeValue;

public class GreyTalonFX extends TalonFX {
  private final TalonFX m_motor;

  public GreyTalonFX(int deviceNumber) {
    this(deviceNumber, "");
  }

  public GreyTalonFX(int deviceNumber, String canbus) {
    super(deviceNumber, canbus);

    m_motor = new TalonFX(deviceNumber, canbus);
    // Factory Default
    var motorConfig = new TalonFXConfiguration();

    // Audio Config
    motorConfig.Audio.BeepOnBoot = true;

    motorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // Motor Directions
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Neutral Mode
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.PeakForwardDutyCycle = 0.0;
    motorConfig.MotorOutput.PeakReverseDutyCycle = 0.0;

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = 0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorConfig.CurrentLimits.StatorCurrentLimit = 0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    motorConfig.CustomParams.CustomParam0 = 0;
    motorConfig.CustomParams.CustomParam1 = 0;

    // Motor feedback
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfig.Feedback.FeedbackRemoteSensorID = 0;
    motorConfig.Feedback.FeedbackRotorOffset = 0;
    motorConfig.Feedback.RotorToSensorRatio = 0.0;
    motorConfig.Feedback.SensorToMechanismRatio = 0.0;

    motorConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
    motorConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0.0;
    motorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 0;
    motorConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    motorConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
    motorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
    motorConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = 0;
    motorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    motorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    motorConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.0;
    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;

    // Ramp rate
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;

    // Position PID Parameters
    motorConfig.Slot0.kP = 0.0;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;
    motorConfig.Slot0.kS = 0.0;
    motorConfig.Slot0.kV = 0.0;

    motorConfig.Slot1.kP = 0.0;
    motorConfig.Slot1.kI = 0.0;
    motorConfig.Slot1.kD = 0.0;
    motorConfig.Slot1.kS = 0.0;
    motorConfig.Slot1.kV = 0.0;

    motorConfig.Slot2.kP = 0.0;
    motorConfig.Slot2.kI = 0.0;
    motorConfig.Slot2.kD = 0.0;
    motorConfig.Slot2.kS = 0.0;
    motorConfig.Slot2.kV = 0.0;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    motorConfig.MotionMagic.MotionMagicAcceleration = 0.0;
    motorConfig.MotionMagic.MotionMagicJerk = 0.0;

    // Apply configurator
    m_motor.getConfigurator().apply(motorConfig);
  }
}
