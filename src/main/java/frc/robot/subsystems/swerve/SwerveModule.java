package frc.robot.subsystems.swerve;

import frc.robot.devices.GreyTalonFX;
import frc.robot.devices.GreyTalonFX.ControlMode;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.SwerveMath;
import frc.robot.shared.SwerveModuleConfig;
import frc.robot.shared.SwerveModuleState2;
import frc.robot.shared.mechanisms.GearedMechanism;
import frc.robot.shared.mechanisms.LinearMechanism;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
  public final int moduleNumber;
  private final Rotation2d m_angleOffset;
  private final GreyTalonFX m_angleMotor;
  private final GreyTalonFX m_driveMotor;
  private final CANcoder m_angleEncoder;
  private final LinearMechanism m_driveMechanism =
      new LinearMechanism(DriveInfo.DRIVE_GEAR_RATIO, DriveInfo.WHEEL_DIAMETER_METERS);
  private final GearedMechanism m_angleMechanism = new GearedMechanism(DriveInfo.ANGLE_GEAR_RATIO);
  private SwerveModuleState2 m_lastState;

  private final TalonFXConfiguration m_driveMotorConfig;

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

    m_driveMotorConfig.Slot0.kP = DriveInfo.DRIVE_KP;
    m_driveMotorConfig.Slot0.kI = DriveInfo.DRIVE_KI;
    m_driveMotorConfig.Slot0.kD = DriveInfo.DRIVE_KD;
    m_driveMotorConfig.Slot0.kV = DriveInfo.DRIVE_KF;

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
    m_angleMotor.setRotorPositionRotation2d(
        m_angleMechanism.getRotorRotationFromOutputRotation(getCanCoder().minus(m_angleOffset)));
  }

  public SwerveModuleState2 getState() {
    double velocityInMPS =
        m_driveMechanism.getOutputDistanceFromRotorRotation(
            m_driveMotor.getRotorVelocityRotation2d());

    return new SwerveModuleState2(
        velocityInMPS, getAngleMotorRotation2d(), getAngleMotorVelocityRotation2d());
  }

  public Rotation2d getAngleMotorRotation2d() {
    return m_angleMechanism.getOutputRotationFromRotorRotation(
        m_angleMotor.getRotorPositionRotation2d());
  }

  public Rotation2d getAngleMotorVelocityRotation2d() {
    return m_angleMechanism.getOutputRotationFromRotorRotation(
        m_angleMotor.getRotorVelocityRotation2d());
  }

  public double getDriveMotorMeters() {
    return m_driveMechanism.getOutputDistanceFromRotorRotation(
        m_driveMotor.getRotorPositionRotation2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMotorMeters(), getState().angle);
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The desired state of the module.
   */
  public void setDesiredState(SwerveModuleState2 desiredState) {
    setDesiredState(desiredState, false);
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The desired state of the module.
   * @param force If true, the module will be set to the desired state regardless of the current
   *     state. Disables optimizations such as anti-jitter.
   */
  public void setDesiredState(SwerveModuleState2 desiredState, boolean force) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // CTRE is not
    desiredState =
        SwerveMath.optimize(
            desiredState,
            getState().angle,
            Rotation2d.fromRadians(
                    m_lastState.omegaRotationPerSecond.getRadians() * DriveInfo.ANGLE_KV)
                .times(1.0)); // 0.06 is a fudge number

    Rotation2d desiredFalconVelocityInRPS =
        m_driveMechanism.getRotorRotationFromOutputDistance(desiredState.speedMetersPerSecond);

    if (desiredState.speedMetersPerSecond != m_lastState.speedMetersPerSecond) {
      m_driveMotor.setControl(
          ControlMode.VelocityVoltage, desiredFalconVelocityInRPS.getRotations());
    }

    // If we are forcing the angle
    if (!force) {
      // Prevent rotating module if speed is less than 1%. Prevents jittering.
      SwerveMath.antiJitter(desiredState, m_lastState, DriveInfo.MAX_VELOCITY_METERS_PER_SECOND);
    }

    // Prevent module rotation if angle is the same as the previous angle.
    if (desiredState.angle != m_lastState.angle) {
      m_angleMotor.setControl(
          ControlMode.PositionVoltage,
          m_angleMechanism.getRotorRotationFromOutputRotation(desiredState.angle).getRotations());
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
