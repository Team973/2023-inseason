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

  private final GreyTalonFX m_bottomRollerMotor;
  private final GreyTalonFX m_topRollerMotor;

  private final DigitalInput m_coneSensor;

  private GamePiece m_lastGamePiece = GamePiece.None;
  @Getter private boolean m_hasGamePiece = false;

  private double m_bottomRollerStator = 0.0;
  private double m_topRollerStator = 0.0;
  private double m_bottomRollerOutput = 0.0;
  private double m_topRollerOutput = 0.0;
  private double m_statorCurrentLimit = 70.0;
  private double m_supplyCurrentLimit = 100.0;

  public enum IntakeState {
    In,
    Out,
    Hold,
    Neutral
  }

  public Claw() {
    m_bottomRollerMotor = new GreyTalonFX(ClawInfo.BOTTOM_ROLLER_FX_ID, RobotInfo.CANIVORE_NAME);
    m_topRollerMotor = new GreyTalonFX(ClawInfo.TOP_ROLLER_FX_ID, RobotInfo.CANIVORE_NAME);
    m_coneSensor = new DigitalInput(ClawInfo.CONE_SENSOR_ID);

    configIntakeMotor();
  }

  private void configIntakeMotor() {
    var motorConfig = new TalonFXConfiguration();

    // Current limits
    motorConfig.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    m_bottomRollerMotor.getConfigurator().apply(motorConfig);
    m_topRollerMotor.getConfigurator().apply(motorConfig);
  }

  private boolean checkForGamePiece() {
    boolean gamePiece = false;

    // If we have a cube, we want to check the stator current of the bottom roller
    if (Robot.getCurrentGamePiece() == GamePiece.Cube) {
      gamePiece = Math.abs(m_bottomRollerStator) > m_statorCurrentLimit - 10.0;
    }
    // If we have a cone, we want to check the banner sensor and the average stator
    // current of the top and bottom rollers
    else {
      gamePiece =
          getConeSensor()
              && Math.abs((m_bottomRollerStator + m_topRollerStator) / 2.0)
                  > m_statorCurrentLimit - 10.0;
    }

    if (gamePiece) {
      m_hasGamePiece = true;
    }
    return m_hasGamePiece;
  }

  private boolean getConeSensor() {
    return m_coneSensor.get();
  }

  public void dashboardUpdate() {
    SmartDashboard.putNumber("Intake Stator", m_bottomRollerStator);
    SmartDashboard.putNumber("Intake Supply", m_bottomRollerMotor.getSupplyCurrent().getValue());
    SmartDashboard.putNumber("Intake Velocity", m_bottomRollerMotor.getVelocity().getValue());
    SmartDashboard.putBoolean("Game Piece", m_hasGamePiece);
    SmartDashboard.putBoolean("Cone Sensor", getConeSensor());
  }

  public void update() {
    GamePiece currentGamePiece = Robot.getCurrentGamePiece();
    m_bottomRollerStator = m_bottomRollerMotor.getStatorCurrent().getValue();
    m_topRollerStator = m_topRollerMotor.getStatorCurrent().getValue();
    checkForGamePiece();

    if (currentGamePiece != m_lastGamePiece || currentGamePiece == GamePiece.None) {
      m_hasGamePiece = false;
    }

    switch (m_intakeState) {
      case In:
        if (currentGamePiece == GamePiece.Cube) {
          m_bottomRollerOutput = -0.5;
        } else {
          m_bottomRollerOutput = 0.8;
        }
        break;
      case Out:
        m_hasGamePiece = false;
        if (currentGamePiece == GamePiece.Cube) {
          m_bottomRollerOutput = 0.3;
        } else {
          m_bottomRollerOutput = -1.0;
        }
        break;
      case Hold:
        if (currentGamePiece == GamePiece.Cube) {
          m_bottomRollerOutput = -0.1;
        } else {
          m_bottomRollerOutput = 0.1;
        }
        break;
      case Neutral:
      default:
        m_bottomRollerOutput = 0.0;
        m_topRollerOutput = 0.0;
        break;
    }

    // If we have a cone, we want the top roller to be the same speed as the bottom
    // in the opposite direction (by default). If we have a cube, we want the top
    // roller to not move.
    if (currentGamePiece == GamePiece.Cube) {
      m_topRollerOutput = 0.0;
    } else {
      m_topRollerOutput = m_bottomRollerOutput;
    }

    m_bottomRollerMotor.set(m_bottomRollerOutput);
    m_topRollerMotor.set(m_topRollerOutput);

    m_lastGamePiece = currentGamePiece;
  }

  public void reset() {
    setIntakeState(IntakeState.Neutral);
  }
}
