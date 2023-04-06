package frc.robot.subsystems;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.Robot;
import frc.robot.devices.GreyTalonFX;
import frc.robot.devices.GreyTalonFX.ControlMode;
import frc.robot.shared.RobotInfo;
import frc.robot.shared.Subsystem;
import frc.robot.subsystems.Superstructure.GamePiece;

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

    configIntakeMotor();
  }

  private void configIntakeMotor() {
    var motorConfig = m_intakeMotor.getCurrentConfig();

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    m_intakeMotor.setConfig(motorConfig);
  }

  private boolean checkForGamePiece() {
    boolean atStatorLimit = Math.abs(m_intakeStator) > m_statorCurrentLimit - 10.0;
    boolean check = atStatorLimit;
    if (Robot.getCurrentGamePiece() == GamePiece.Cone) {
      check = getConeSensor() && atStatorLimit;
    }

    if (check) {
      m_hasGamePiece = true;
    }
    return m_hasGamePiece;
  }

  private boolean getConeSensor() {
    return m_coneSensor.get();
  }

  public void dashboardUpdate() {}

  public void debugDashboardUpdate() {
    SmartDashboard.putNumber("Intake Stator", m_intakeStator);
    SmartDashboard.putNumber("Intake Supply", m_intakeMotor.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("Intake Velocity", m_intakeMotor.getVelocity().getValue());
    SmartDashboard.putBoolean("Game Piece", m_hasGamePiece);
    SmartDashboard.putBoolean("Cone Sensor", getConeSensor());
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

    m_intakeMotor.setControl(ControlMode.DutyCycleOut, m_intakeMotorOutput);

    m_lastGamePiece = currentGamePiece;
  }

  public void reset() {
    setIntakeState(IntakeState.Neutral);
  }
}
