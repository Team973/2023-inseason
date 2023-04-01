package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.Robot;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.shared.GreyTalonFX;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.Subsystem;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

@Accessors(prefix = "m_")
public class Claw implements Subsystem {
  @Setter @Getter private IntakeState m_intakeState = IntakeState.Neutral;

  private final GreyTalonFX m_intakeMotor;

  private final DigitalInput m_coneSensor;
  private final DigitalInput m_cubeSensor;

  private GamePiece m_lastGamePiece = GamePiece.None;
  @Getter private boolean m_hasGamePiece = false;

  private double m_intakeStator = 0.0;
  private double m_intakeMotorOutput = 0.0;
  private double m_statorCurrentLimit = 70.0;
  private double m_supplyCurrentLimit = 100.0;

  public enum IntakeState {
    In,
    Out,
    Hold,
    Neutral
  }

  public Claw() {
    m_intakeMotor = new GreyTalonFX(ClawInfo.INTAKE_FX_ID, RobotInfo.CANIVORE_NAME);
    m_coneSensor = new DigitalInput(ClawInfo.CONE_SENSOR_ID);
    m_cubeSensor = new DigitalInput(ClawInfo.CUBE_SENSOR_ID);

    configIntakeMotor();
  }

  private void configIntakeMotor() {
    var motorConfig = new TalonFXConfiguration();

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    m_intakeMotor.getConfigurator().apply(motorConfig);
  }

  private boolean checkForGamePiece() {
    boolean atStatorLimit = Math.abs(m_intakeStator) > m_statorCurrentLimit - 10.0;
    boolean check = atStatorLimit;
    if (Robot.getCurrentGamePiece() == GamePiece.Cone) {
      check = getConeSensor() && atStatorLimit;
    } else if (Robot.getCurrentGamePiece() == GamePiece.Cube) {
      check = getCubeSensor() && atStatorLimit;
    }

    if (check) {
      m_hasGamePiece = true;
    }
    return m_hasGamePiece;
  }

  private boolean getConeSensor() {
    return m_coneSensor.get();
  }

  private boolean getCubeSensor() {
    return m_cubeSensor.get();
  }

  public void dashboardUpdate() {
    SmartDashboard.putNumber("Intake Stator", m_intakeStator);
    SmartDashboard.putNumber("Intake Supply", m_intakeMotor.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("Intake Velocity", m_intakeMotor.getVelocity().getValue());
    SmartDashboard.putBoolean("Game Piece", m_hasGamePiece);
    SmartDashboard.putBoolean("Cone Sensor", getConeSensor());
    SmartDashboard.putBoolean("Cube Sensor", getCubeSensor());
  }

  public void update() {
    GamePiece currentGamePiece = Robot.getCurrentGamePiece();
    m_intakeStator = m_intakeMotor.getStatorCurrent().getValue();
    checkForGamePiece();

    if (currentGamePiece != m_lastGamePiece || currentGamePiece == GamePiece.None) {
      m_hasGamePiece = false;
    }

    switch (m_intakeState) {
      case In:
        if (currentGamePiece == GamePiece.Cube) {
          m_intakeMotorOutput = -0.5;
        } else {
          m_intakeMotorOutput = 0.8;
        }
        break;
      case Out:
        m_hasGamePiece = false;
        if (currentGamePiece == GamePiece.Cube) {
          m_intakeMotorOutput = 0.3;
        } else {
          m_intakeMotorOutput = -1.0;
        }
        break;
      case Hold:
        if (currentGamePiece == GamePiece.Cube) {
          m_intakeMotorOutput = -0.1;
        } else {
          m_intakeMotorOutput = 0.1;
        }
        break;
      case Neutral:
      default:
        m_intakeMotorOutput = 0.0;
        break;
    }

    m_intakeMotor.set(m_intakeMotorOutput);

    m_lastGamePiece = currentGamePiece;
  }

  public void reset() {
    setIntakeState(IntakeState.Neutral);
  }
}
