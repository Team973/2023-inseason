package frc.robot.devices;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.ForwardLimitSourceValue;
import com.ctre.phoenixpro.signals.ForwardLimitTypeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.ReverseLimitSourceValue;
import com.ctre.phoenixpro.signals.ReverseLimitTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Data;
import lombok.Getter;
import lombok.experimental.Accessors;

/** A GreyTalonFX is a TalonFX with a default configuration. */
@Accessors(prefix = "m_")
public class GreyTalonFX extends TalonFX {
  // see:
  // https://docs.google.com/spreadsheets/d/1cdySrJRMEgjMhgvOm5zbGFtuejurvd-CIxzr0Xd3ua8/edit#gid=0
  private static final double FOC_INTERCEPT_RPM = 5088.5;
  private boolean m_lastOptimizedFOC = true;

  public enum ControlMode {
    DutyCycleOut,
    MotionMagicDutyCycle,
    MotionMagicVoltage,
    PositionDutyCycle,
    PositionVoltage,
    VelocityDutyCycle,
    VelocityVoltage
  }

  @Data
  public class OutputParams {
    private final ControlMode m_controlMode;
    private final double m_demand;
    private final boolean m_enableFOC;
    private final double m_feedForward;
    private final int m_slot;
    private final boolean m_overrideBrakeDurNeutral;
  }

  /**
   * Create a GreyTalonFX.
   *
   * @param deviceNumber TalonFX device number
   */
  public GreyTalonFX(int deviceNumber) {
    this(deviceNumber, "");
  }

  /**
   * Create a GreyTalonFX.
   *
   * @param deviceNumber TalonFX device number
   * @param canbus CAN bus name
   */
  public GreyTalonFX(int deviceNumber, String canbus) {
    super(deviceNumber, canbus);
    factoryDefault();
  }

  @Getter private TalonFXConfiguration m_currentConfig;
  private OutputParams m_lastOutputParams;
  private StatusCode m_lastControlCode;

  /** Factory default the TalonFX. */
  public void factoryDefault() {
    // Factory Default
    var motorConfig = new TalonFXConfiguration();

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
    motorConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    motorConfig.MotorOutput.PeakReverseDutyCycle = -1.0;

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
   * Set the configuration of the TalonFX.
   *
   * @param config The configuration to apply.
   */
  public void setConfig(TalonFXConfiguration config) {
    this.getConfigurator().apply(config);
    m_currentConfig = config;
  }

  /**
   * @deprecated Use {@link #getCurrentConfig()} and {@link #setConfig(TalonFXConfiguration)}
   *     instead.
   */
  @Deprecated
  public TalonFXConfigurator getConfigurator() {
    return super.getConfigurator();
  }

  /**
   * Get the current position Rotation2d of the TalonFX.
   *
   * @return The current position Rotation2d of the TalonFX.
   */
  public Rotation2d getRotorPositionRotation2d() {
    return Rotation2d.fromRotations(getRotorPosition().getValue());
  }

  /**
   * Get the current velocity Rotation2d of the TalonFX.
   *
   * @return The current velocity Rotation2d of the TalonFX.
   */
  public Rotation2d getRotorVelocityRotation2d() {
    return Rotation2d.fromRotations(getRotorVelocity().getValue());
  }

  /**
   * Set the current position Rotation2d of the TalonFX.
   *
   * @return Status Code of the request, 0 is OK.
   */
  public StatusCode setRotorPositionRotation2d(Rotation2d position) {
    return setRotorPosition(position.getRotations());
  }

  /**
   * Determine whether trap or FOC should be used.
   *
   * <p>FOC is used when the motor is below the intercept RPM, and trap is used when the motor is
   * above the intercept RPM. The intercept RPM is determined by the FOC_INTERCEPT_RPM constant. It
   * will only switch between trap and FOC when the motor is more than 2% outside of the intercept
   * RPM.
   *
   * @return Whether trap or FOC should be used.
   */
  public boolean optimizedFOC() {
    double speedRPM = getRotorVelocity().getValue() * 60.0;
    double tolerance = FOC_INTERCEPT_RPM * 0.02;

    if (m_lastOptimizedFOC) {
      if (speedRPM >= FOC_INTERCEPT_RPM + tolerance) {
        m_lastOptimizedFOC = false;
      }
    } else {
      if (speedRPM <= FOC_INTERCEPT_RPM - tolerance) {
        m_lastOptimizedFOC = true;
      }
    }

    return m_lastOptimizedFOC;
  }

  /**
   * Set the output of the TalonFX with optimized FOC.
   *
   * <p>By default FOC is enabled, Feedforward is disabled, and PID Slot 0 is used
   *
   * @param controlMode The control mode to use.
   * @param demand The demand to use.
   * @return Status Code of the request, 0 is OK.
   */
  public StatusCode setControl(ControlMode controlMode, double demand) {
    return setControl(controlMode, demand, optimizedFOC(), 0.0, 0, false);
  }

  /**
   * Set the output of the TalonFX with optimized FOC.
   *
   * @param controlMode The control mode to use.
   * @param demand The demand to use.
   * @param pidSlot The PID slot to use.
   * @return Status Code of the request, 0 is OK.
   */
  public StatusCode setControl(ControlMode controlMode, double demand, int pidSlot) {
    return setControl(controlMode, demand, optimizedFOC(), 0.0, pidSlot, false);
  }

  /**
   * Set the output of the TalonFX with optimized FOC.
   *
   * <p>By default PID Slot 0 is used
   *
   * @param controlMode The control mode to use.
   * @param demand The demand to use.
   * @param feedForward The feed forward to use.
   * @return Status Code of the request, 0 is OK.
   */
  public StatusCode setControl(ControlMode controlMode, double demand, double feedForward) {
    return setControl(controlMode, demand, optimizedFOC(), feedForward, 0, false);
  }

  /**
   * Set the output of the TalonFX.
   *
   * <p>By default Feedforward is disabled and PID Slot 0 is used
   *
   * @param controlMode The control mode to use.
   * @param demand The demand to use.
   * @param enableFOC Set to true to use FOC commutation, which increases peak power by ~15%. Set to
   *     false to use trapezoidal commutation. FOC improves motor performance by leveraging torque
   *     (current) control. However, this may be inconvenient for applications that require
   *     specifying duty cycle or voltage. CTR-Electronics has developed a hybrid method that
   *     combines the performances gains of FOC while still allowing applications to provide
   *     duty-cycle or voltage demand. This not to be confused with simple sinusoidal control or
   *     phase voltage control which lacks the performance gains.
   * @return Status Code of the request, 0 is OK.
   */
  public StatusCode setControl(ControlMode controlMode, double demand, boolean enableFOC) {
    return setControl(controlMode, demand, enableFOC, 0.0, 0, false);
  }

  /**
   * Set the output of the TalonFX.
   *
   * <p>By default PID Slot 0 is used
   *
   * @param controlMode The control mode to use.
   * @param demand The demand to use.
   * @param enableFOC Set to true to use FOC commutation, which increases peak power by ~15%. Set to
   *     false to use trapezoidal commutation. FOC improves motor performance by leveraging torque
   *     (current) control. However, this may be inconvenient for applications that require
   *     specifying duty cycle or voltage. CTR-Electronics has developed a hybrid method that
   *     combines the performances gains of FOC while still allowing applications to provide
   *     duty-cycle or voltage demand. This not to be confused with simple sinusoidal control or
   *     phase voltage control which lacks the performance gains.
   * @param feedForward The feed forward to use.
   * @return Status Code of the request, 0 is OK.
   */
  public StatusCode setControl(
      ControlMode controlMode, double demand, boolean enableFOC, double feedForward) {
    return setControl(controlMode, demand, enableFOC, feedForward, 0, false);
  }

  /**
   * Set the output of the TalonFX.
   *
   * @param controlMode The control mode to use.
   * @param demand The demand to use.
   * @param enableFOC Set to true to use FOC commutation, which increases peak power by ~15%. Set to
   *     false to use trapezoidal commutation. FOC improves motor performance by leveraging torque
   *     (current) control. However, this may be inconvenient for applications that require
   *     specifying duty cycle or voltage. CTR-Electronics has developed a hybrid method that
   *     combines the performances gains of FOC while still allowing applications to provide
   *     duty-cycle or voltage demand. This not to be confused with simple sinusoidal control or
   *     phase voltage control which lacks the performance gains.
   * @param feedForward The feed forward to use.
   * @param slot Select which gains are applied by selecting the slot. Use the configuration api to
   *     set the gain values for the selected slot before enabling this feature. Slot must be within
   *     [0,2].
   * @param overrideBrakeDurNeutral Set to true to static-brake the rotor when output is zero (or
   *     within deadband). Set to false to use the NeutralMode configuration setting (default). This
   *     flag exists to provide the fundamental behavior of this control when output is zero, which
   *     is to provide 0V to the motor.
   * @return Status Code of the request, 0 is OK.
   */
  public StatusCode setControl(
      ControlMode controlMode,
      double demand,
      boolean enableFOC,
      double feedForward,
      int slot,
      boolean overrideBrakeDurNeutral) {

    ControlRequest motorOutput = new DutyCycleOut(0.0);

    var currentOutputParams =
        new OutputParams(
            controlMode, demand, enableFOC, feedForward, slot, overrideBrakeDurNeutral);

    if (m_lastOutputParams == null || !m_lastOutputParams.equals(currentOutputParams)) {
      switch (controlMode) {
        case DutyCycleOut:
          motorOutput = new DutyCycleOut(demand, enableFOC, overrideBrakeDurNeutral);
          break;
        case MotionMagicDutyCycle:
          motorOutput =
              new MotionMagicDutyCycle(
                  demand, enableFOC, feedForward, slot, overrideBrakeDurNeutral);
          break;
        case MotionMagicVoltage:
          motorOutput =
              new MotionMagicVoltage(demand, enableFOC, feedForward, slot, overrideBrakeDurNeutral);
          break;
        case PositionDutyCycle:
          motorOutput =
              new PositionDutyCycle(demand, enableFOC, feedForward, slot, overrideBrakeDurNeutral);
          break;
        case PositionVoltage:
          motorOutput =
              new PositionVoltage(demand, enableFOC, feedForward, slot, overrideBrakeDurNeutral);
          break;
        case VelocityDutyCycle:
          motorOutput =
              new VelocityDutyCycle(demand, enableFOC, feedForward, slot, overrideBrakeDurNeutral);
          break;
        case VelocityVoltage:
          motorOutput =
              new VelocityVoltage(demand, enableFOC, feedForward, slot, overrideBrakeDurNeutral);
          break;
        default:
          // Uses the above DutyCycleOut set to 0.0
          break;
      }

      m_lastOutputParams = currentOutputParams;
      m_lastControlCode = super.setControl(motorOutput);
    }
    return m_lastControlCode;
  }

  @Deprecated
  public void set(double speed) {
    super.set(speed);
  }

  /**
   * @deprecated Use {@link #setControl(ControlMode, double, double)} instead.
   */
  @Deprecated
  public StatusCode setControl(ControlRequest controlRequest) {
    return super.setControl(controlRequest);
  }

  /**
   * @deprecated Use {@link #setControl(ControlMode, double, double)} instead.
   */
  @Deprecated
  public StatusCode setControl(DutyCycleOut request) {
    return super.setControl(request);
  }

  /**
   * @deprecated Use {@link #setControl(ControlMode, double, double)} instead.
   */
  @Deprecated
  public StatusCode setControl(VoltageOut request) {
    return super.setControl(request);
  }

  /**
   * @deprecated Use {@link #setControl(ControlMode, double, double)} instead.
   */
  @Deprecated
  public StatusCode setControl(PositionDutyCycle request) {
    return super.setControl(request);
  }

  /**
   * @deprecated Use {@link #setControl(ControlMode, double, double)} instead.
   */
  @Deprecated
  public StatusCode setControl(PositionVoltage request) {
    return super.setControl(request);
  }

  /**
   * @deprecated Use {@link #setControl(ControlMode, double, double)} instead.
   */
  @Deprecated
  public StatusCode setControl(VelocityDutyCycle request) {
    return super.setControl(request);
  }

  /**
   * @deprecated Use {@link #setControl(ControlMode, double, double)} instead.
   */
  @Deprecated
  public StatusCode setControl(VelocityVoltage request) {
    return super.setControl(request);
  }

  /**
   * @deprecated Use {@link #setControl(ControlMode, double, double)} instead.
   */
  @Deprecated
  public StatusCode setControl(MotionMagicDutyCycle request) {
    return super.setControl(request);
  }

  /**
   * @deprecated Use {@link #setControl(ControlMode, double, double)} instead.
   */
  @Deprecated
  public StatusCode setControl(MotionMagicVoltage request) {
    return super.setControl(request);
  }
}
