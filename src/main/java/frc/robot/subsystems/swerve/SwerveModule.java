package frc.robot.subsystems.swerve;

import frc.robot.shared.GreyTalonFX;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.SwerveModuleConfig;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;
  private GreyTalonFX m_angleMotor;
  private GreyTalonFX m_driveMotor;
  private CANcoder m_angleEncoder;
  private Rotation2d lastAngle;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(DriveInfo.driveKS, DriveInfo.driveKV, DriveInfo.driveKA);

  private PositionVoltage m_anglePosition = new PositionVoltage(0.0);

  private VelocityVoltage m_driveVelocity =
      new VelocityVoltage(0.0, true, feedforward.calculate(0.0), 0, false);

  public SwerveModule(int moduleNumber, SwerveModuleConfig moduleConfig) {
    this.moduleNumber = moduleNumber;
    angleOffset = Rotation2d.fromDegrees(moduleConfig.angleOffset);

    /* Angle Encoder Config */
    m_angleEncoder = new CANcoder(moduleConfig.cancoderID, RobotInfo.CANIVORE_NAME);
    configAngleEncoder();

    /* Angle Motor Config */
    m_angleMotor = new GreyTalonFX(moduleConfig.angleMotorID, RobotInfo.CANIVORE_NAME);
    configAngleMotor();

    /* Drive Motor Config */
    m_driveMotor = new GreyTalonFX(moduleConfig.driveMotorID, RobotInfo.CANIVORE_NAME);
    configDriveMotor();

    BaseStatusSignalValue.waitForAll(0.5, m_angleEncoder.getAbsolutePosition());
    resetToAbsolute();

    lastAngle = getState().angle;
  }

  private void configAngleEncoder() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    m_angleEncoder.getConfigurator().apply(encoderConfig);
  }

  private void configAngleMotor() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motorConfig.Slot0.kP = 1.5;
    motorConfig.Slot0.kI = 0.4;
    motorConfig.Slot0.kD = 0.3;
    motorConfig.Slot0.kS = 0.2;

    motorConfig.CurrentLimits.StatorCurrentLimit = 150.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 150.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_angleMotor.getConfigurator().apply(motorConfig);

    resetToAbsolute();
  }

  private void configDriveMotor() {
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.Slot0.kP = 0.5;
    motorConfig.Slot0.kI = 0.04;
    motorConfig.Slot0.kD = 0.06;
    motorConfig.Slot0.kV = 0.1;

    motorConfig.CurrentLimits.StatorCurrentLimit = 100.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_driveMotor.getConfigurator().apply(motorConfig);
    m_driveMotor.setRotorPosition(0.0);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValue());
  }

  public void resetToAbsolute() {
    double absolutePosition =
        (getCanCoder().minus(angleOffset).getRotations()) * DriveInfo.ANGLE_GEAR_RATIO;
    m_angleMotor.setRotorPosition(absolutePosition);
  }

  public SwerveModuleState2 getState() {
    double wheelRPS = m_driveMotor.getRotorVelocity().getValue() / DriveInfo.DRIVE_GEAR_RATIO;
    double velocityInMPS = wheelRPS * DriveInfo.WHEEL_CIRCUMFRENCE_METERS;

    Rotation2d angle =
        Rotation2d.fromRotations(
            m_angleMotor.getRotorPosition().getValue() / DriveInfo.ANGLE_GEAR_RATIO);

    Rotation2d angularVelocity =
        Rotation2d.fromRotations(
            m_angleMotor.getRotorVelocity().getValue() / DriveInfo.ANGLE_GEAR_RATIO);

    return new SwerveModuleState2(velocityInMPS, angle, angularVelocity.getRadians());
  }

  public double getAngleRaw() {
    return m_angleMotor.getRotorPosition().getValue();
  }

  public SwerveModulePosition getPosition() {
    double wheelRotations = m_driveMotor.getRotorPosition().getValue() / DriveInfo.DRIVE_GEAR_RATIO;
    double wheelDistanceMeters = wheelRotations * DriveInfo.WHEEL_CIRCUMFRENCE_METERS;
    return new SwerveModulePosition(wheelDistanceMeters, getState().angle);
  }

  public void setDesiredState(SwerveModuleState2 desiredState) {
    desiredState =
        CTREModuleState.optimize(
            desiredState,
            getState().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which CTRE is not

    double desiredWheelVelocityInRPS =
        desiredState.speedMetersPerSecond / DriveInfo.WHEEL_CIRCUMFRENCE_METERS;
    double desiredFalconVelocityInRPS = desiredWheelVelocityInRPS * DriveInfo.DRIVE_GEAR_RATIO;
    m_driveMotor.setControl(
        m_driveVelocity
            .withVelocity(desiredFalconVelocityInRPS)
            .withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)));

    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (DriveInfo.MAX_VELOCITY_METERS_PER_SECOND * 0.01))
            ? lastAngle
            : desiredState.angle;

    m_angleMotor.setControl(
        m_anglePosition.withPosition(angle.getRotations() * DriveInfo.ANGLE_GEAR_RATIO));
    lastAngle = angle;
  }
}
