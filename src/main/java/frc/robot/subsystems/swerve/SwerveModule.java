package frc.robot.subsystems.swerve;

import frc.robot.devices.GreyTalonFX;
import frc.robot.devices.GreyTalonFX.ControlMode;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.SwerveModuleConfig;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  public final int moduleNumber;
  private final Rotation2d m_angleOffset;
  private final GreyTalonFX m_angleMotor;
  private final GreyTalonFX m_driveMotor;
  private final CANcoder m_angleEncoder;
  private SwerveModuleState m_lastState;

  private final TalonFXConfiguration m_driveMotorConfig;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(DriveInfo.driveKS, DriveInfo.driveKV, DriveInfo.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConfig moduleConfig) {
    this.moduleNumber = moduleNumber;
    m_angleOffset = Rotation2d.fromDegrees(moduleConfig.angleOffset);

    /* Angle Encoder Config */
    m_angleEncoder = new CANcoder(moduleConfig.cancoderID, RobotInfo.CANIVORE_NAME);
    configAngleEncoder();

    /* Angle Motor Config */
    m_angleMotor = new GreyTalonFX(moduleConfig.angleMotorID, RobotInfo.CANIVORE_NAME);
    configAngleMotor();

    /* Drive Motor Config */
    m_driveMotor = new GreyTalonFX(moduleConfig.driveMotorID, RobotInfo.CANIVORE_NAME);
    m_driveMotorConfig = m_driveMotor.getCurrentConfig();
    configDriveMotor();

    BaseStatusSignalValue.waitForAll(0.5, m_angleEncoder.getAbsolutePosition());
    resetToAbsolute();

    m_lastState = getState();
  }

  private void configAngleEncoder() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    m_angleEncoder.getConfigurator().apply(encoderConfig);
  }

  private void configAngleMotor() {
    var motorConfig = m_angleMotor.getCurrentConfig();

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motorConfig.Slot0.kP = DriveInfo.ANGLE_KP;
    motorConfig.Slot0.kI = DriveInfo.ANGLE_KI;
    motorConfig.Slot0.kD = DriveInfo.ANGLE_KD;
    motorConfig.Slot0.kS = DriveInfo.ANGLE_KF;

    motorConfig.CurrentLimits.StatorCurrentLimit = 150.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 150.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_angleMotor.setConfig(motorConfig);

    resetToAbsolute();
  }

  private void configDriveMotor() {
    m_driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_driveMotorConfig.Slot0.kP = 0.00973;
    m_driveMotorConfig.Slot0.kI = 0.0;
    m_driveMotorConfig.Slot0.kD = 0.0;
    m_driveMotorConfig.Slot0.kV = 0.00973;

    m_driveMotorConfig.CurrentLimits.StatorCurrentLimit = 100.0;
    m_driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    m_driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
    m_driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_driveMotor.setConfig(m_driveMotorConfig);
    m_driveMotor.setRotorPosition(0.0);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValue());
  }

  public void resetToAbsolute() {
    double absolutePosition =
        (getCanCoder().minus(m_angleOffset).getRotations()) * DriveInfo.ANGLE_GEAR_RATIO;
    m_angleMotor.setRotorPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    double wheelRPS = m_driveMotor.getRotorVelocity().getValue() / DriveInfo.DRIVE_GEAR_RATIO;
    double velocityInMPS = wheelRPS * DriveInfo.WHEEL_CIRCUMFERENCE_METERS;

    Rotation2d angle =
        Rotation2d.fromRotations(
            m_angleMotor.getRotorPosition().getValue() / DriveInfo.ANGLE_GEAR_RATIO);

    return new SwerveModuleState(velocityInMPS, angle);
  }

  public double getAngleRaw() {
    return m_angleMotor.getRotorPosition().getValue();
  }

  public double getDriveMotorMeters() {
    double wheelRotations = m_driveMotor.getRotorPosition().getValue() / DriveInfo.DRIVE_GEAR_RATIO;
    return wheelRotations * DriveInfo.WHEEL_CIRCUMFERENCE_METERS;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMotorMeters(), getState().angle);
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The desired state of the module.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    setDesiredState(desiredState, false);
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The desired state of the module.
   * @param force If true, the module will be set to the desired state regardless of the current
   *     state. Disables optimizations such as anti-jitter.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean ignoreJitter) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // CTRE is not
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);

    double desiredWheelVelocityInRPS =
        desiredState.speedMetersPerSecond / DriveInfo.WHEEL_CIRCUMFERENCE_METERS;
    double desiredFalconVelocityInRPS = desiredWheelVelocityInRPS * DriveInfo.DRIVE_GEAR_RATIO;

    if (desiredState.speedMetersPerSecond != m_lastState.speedMetersPerSecond) {
      m_driveMotor.setControl(
          ControlMode.VelocityDutyCycle,
          desiredFalconVelocityInRPS,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    if (!ignoreJitter) {
      desiredState.angle =
          (Math.abs(desiredState.speedMetersPerSecond)
                  <= (DriveInfo.MAX_VELOCITY_METERS_PER_SECOND * 0.01))
              ? m_lastState.angle
              : desiredState.angle;
    }

    if (desiredState.angle != m_lastState.angle) {
      m_angleMotor.setControl(
          ControlMode.PositionDutyCycle,
          desiredState.angle.getRotations() * DriveInfo.ANGLE_GEAR_RATIO);
    }

    m_lastState = desiredState;
  }

  public void driveBrake() {
    m_driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_driveMotor.setConfig(m_driveMotorConfig);
  }

  public void driveNeutral() {
    m_driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_driveMotor.setConfig(m_driveMotorConfig);
  }
}
