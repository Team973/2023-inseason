package frc.robot.subsystems.swerve;

import frc.robot.shared.GreyTalonFX;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.RobotInfo.DriveInfo;
import frc.robot.shared.SwerveModuleConfig;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
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
  public int moduleNumber;
  private Rotation2d angleOffset;
  private GreyTalonFX m_angleMotor;
  private GreyTalonFX m_driveMotor;
  private CANcoder m_angleEncoder;
  private Rotation2d lastAngle;

  private final TalonFXConfiguration m_driveMotorConfig;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(DriveInfo.driveKS, DriveInfo.driveKV, DriveInfo.driveKA);

  private PositionDutyCycle m_anglePosition = new PositionDutyCycle(0.0);

  private VelocityDutyCycle m_driveVelocity =
      new VelocityDutyCycle(0.0, true, feedforward.calculate(0.0), 0, false);

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
    m_driveMotorConfig = new TalonFXConfiguration();
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

    motorConfig.Slot0.kP = DriveInfo.ANGLE_KP;
    motorConfig.Slot0.kI = DriveInfo.ANGLE_KI;
    motorConfig.Slot0.kD = DriveInfo.ANGLE_KD;
    motorConfig.Slot0.kS = DriveInfo.ANGLE_KF;

    motorConfig.CurrentLimits.StatorCurrentLimit = 150.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 150.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_angleMotor.getConfigurator().apply(motorConfig);

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

    m_driveMotor.getConfigurator().apply(m_driveMotorConfig);
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

  public SwerveModulePosition getPosition() {
    double wheelRotations = m_driveMotor.getRotorPosition().getValue() / DriveInfo.DRIVE_GEAR_RATIO;
    double wheelDistanceMeters = wheelRotations * DriveInfo.WHEEL_CIRCUMFERENCE_METERS;
    return new SwerveModulePosition(wheelDistanceMeters, getState().angle);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    setDesiredState(desiredState, false);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean ignoreJitter) {
    desiredState =
        CTREModuleState.optimize(
            desiredState,
            getState().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which CTRE is not

    double desiredWheelVelocityInRPS =
        desiredState.speedMetersPerSecond / DriveInfo.WHEEL_CIRCUMFERENCE_METERS;
    double desiredFalconVelocityInRPS = desiredWheelVelocityInRPS * DriveInfo.DRIVE_GEAR_RATIO;
    m_driveMotor.setControl(
        m_driveVelocity
            .withVelocity(desiredFalconVelocityInRPS)
            .withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)));

    Rotation2d angle = desiredState.angle;

    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    if (!ignoreJitter) {
      angle =
          (Math.abs(desiredState.speedMetersPerSecond)
                  <= (DriveInfo.MAX_VELOCITY_METERS_PER_SECOND * 0.01))
              ? lastAngle
              : desiredState.angle;
    }

    m_angleMotor.setControl(
        m_anglePosition.withPosition(angle.getRotations() * DriveInfo.ANGLE_GEAR_RATIO));
    lastAngle = angle;
  }

  public void driveBrake() {
    m_driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_driveMotor.getConfigurator().apply(m_driveMotorConfig);
  }

  public void driveNeutral() {
    m_driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_driveMotor.getConfigurator().apply(m_driveMotorConfig);
  }
}
