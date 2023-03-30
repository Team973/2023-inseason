package frc.robot.devices;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.ForwardLimitSourceValue;
import com.ctre.phoenixpro.signals.ForwardLimitTypeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.ReverseLimitSourceValue;
import com.ctre.phoenixpro.signals.ReverseLimitTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.EqualsAndHashCode;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/** A GreyTalonFX is a TalonFX with a default configuration. */
@Accessors(prefix = "m_")
@EqualsAndHashCode(callSuper = true)
public class GreyTalonFX extends TalonFX {
  @Setter @Getter private double m_gearRatio;

  /**
   * Create a GreyTalonFX.
   *
   * <p>Note: Creates with a gear ratio of 1.0.
   *
   * @param deviceNumber TalonFX device number
   */
  public GreyTalonFX(int deviceNumber) {
    this(deviceNumber, "", 1.0);
  }

  /**
   * Create a GreyTalonFX with a gear ratio.
   *
   * <p>For example, if the pinion is 10 teeth and the gear is 50 teeth, the gear ratio is 1:5 and
   * therefore the value is 0.2.
   *
   * @param deviceNumber TalonFX device number
   * @param gearRatio Gear ratio (output / input, driving:driven)
   */
  public GreyTalonFX(int deviceNumber, double gearRatio) {
    this(deviceNumber, "", gearRatio);
  }

  /**
   * Create a GreyTalonFX.
   *
   * @param deviceNumber TalonFX device number
   * @param canbus CAN bus name
   */
  public GreyTalonFX(int deviceNumber, String canbus) {
    this(deviceNumber, canbus, 1.0);
  }

  /**
   * Create a GreyTalonFX with a gear ratio.
   *
   * <p>For example, if the pinion is 10 teeth and the gear is 50 teeth, the gear ratio is 1:5 and
   * therefore the value is 0.2.
   *
   * @param deviceNumber TalonFX device number
   * @param canbus CAN bus name
   * @param gearRatio Gear ratio (output / input, driving:driven)
   */
  public GreyTalonFX(int deviceNumber, String canbus, double gearRatio) {
    super(deviceNumber, canbus);
    factoryDefault();
    m_gearRatio = gearRatio;
  }

  private GreyTalonFXConfiguration m_currentConfig;

  /** Factory default the TalonFX. */
  public void factoryDefault() {
    // Factory Default
    var motorConfig = new GreyTalonFXConfiguration();

    // Audio Config
    motorConfig.Audio.BeepOnBoot = true;

    motorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // Motor Directions
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Neutral Mode
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
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
    setConfig(motorConfig);
  }

  /**
   * Get the current configuration of the TalonFX.
   *
   * @return A deep copy of the current configuration.
   */
  public GreyTalonFXConfiguration getConfig() {
    return new GreyTalonFXConfiguration(m_currentConfig);
  }

  /**
   * Set the configuration of the TalonFX.
   *
   * @param config The configuration to apply.
   */
  public void setConfig(GreyTalonFXConfiguration config) {
    if (m_currentConfig == null || !m_currentConfig.equals(config)) {
      this.getConfigurator().apply(config);
      m_currentConfig = config;
    }
  }

  /**
   * Get the configurator for the TalonFX.
   *
   * @return The configurator for the TalonFX.
   * @deprecated Use {@link #getConfig()} and {@link #setConfig(GreyTalonFXConfiguration)} instead.
   */
  @Deprecated
  public TalonFXConfigurator getConfigurator() {
    return super.getConfigurator();
  }

  /**
   * Get the current Rotation2d of the TalonFX.
   *
   * @return The current Rotation2d of the TalonFX.
   */
  public Rotation2d getRawPositionRotation2d() {
    return Rotation2d.fromRotations(getRotorPosition().getValue());
  }

  /**
   * Get the current position Rotation2d of the TalonFX through the gear ratio.
   *
   * @return The current position Rotation2d of the TalonFX through the gear ratio.
   */
  public Rotation2d getPositionRotation2d() {
    return getRawPositionRotation2d().times(m_gearRatio);
  }

  /**
   * Get the current velocity Rotation2d of the TalonFX.
   *
   * @return The current velocity Rotation2d of the TalonFX.
   */
  public Rotation2d getRawVelocityRotation2d() {
    return Rotation2d.fromRotations(getRotorVelocity().getValue());
  }

  /**
   * Get the current velocity Rotation2d of the TalonFX through the gear ratio.
   *
   * @return The current velocity Rotation2d of the TalonFX through the gear ratio.
   */
  public Rotation2d getVelocityRotation2d() {
    return getRawVelocityRotation2d().times(m_gearRatio);
  }

  /**
   * Set the current position Rotation2d of the TalonFX.
   *
   * @return The current velocity Rotation2d of the TalonFX.
   */
  public StatusCode setRawRotorPositionRotation2d(Rotation2d position) {
    return setRotorPosition(position.getRotations());
  }

  /**
   * Set the current position Rotation2d of the TalonFX through the gear ratio.
   *
   * @return The current velocity Rotation2d of the TalonFX through the gear ratio.
   */
  public StatusCode setRotorPositionRotation2d(Rotation2d position) {
    return setRawRotorPositionRotation2d(position.div(m_gearRatio));
  }

  /**
   * Convert through the gear ratio to get the rotor's respective Rotation2d.
   *
   * @param rotation The Rotation2d on the output to convert.
   * @return The Rotation2d on the rotor.
   */
  public Rotation2d convertToRotorRotation2d(Rotation2d outputRotation) {
    return outputRotation.div(m_gearRatio);
  }

  /**
   * Convert through the gear ratio to get the output's respective Rotation2d.
   *
   * @param rotation The Rotation2d on the rotor to convert.
   * @return The Rotation2d on the output.
   */
  public Rotation2d convertToOutputRotation2d(Rotation2d rotorRotation) {
    return rotorRotation.times(m_gearRatio);
  }
}
