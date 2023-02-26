// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.shared.RobotInfo.*;

import java.io.FileWriter;
import java.io.PrintWriter;

import frc.robot.AutoManager.AutoMode;
import frc.robot.auto.commands.TrajectoryManager;
import frc.robot.greydash.GreyDashClient;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.shared.Conversions;
import frc.robot.subsystems.CANdleManager;
import frc.robot.subsystems.CANdleManager.LightState;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Claw.WristPreset;
import frc.robot.subsystems.Claw.WristState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.RotationControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Accessors(prefix = "m_")
public class Robot extends TimedRobot {
  @Setter @Getter private static GamePiece m_currentGamePiece = GamePiece.None;

  @Getter private static boolean m_exceptionHappened = false;

  private static boolean m_autoRan = false;

  private static final double GOT_IT_DELAY_MSEC = 300.0;
  private static double m_gotItStartTime;
  private static boolean m_gotIt;

  private final Elevator m_elevator = new Elevator();
  private final Claw m_claw = new Claw();
  private final Drive m_drive = new Drive();
  private final CANdleManager m_candleManager = new CANdleManager();
  private final TrajectoryManager m_trajectoryManager = new TrajectoryManager();
  private final AutoManager m_autoManager =
      new AutoManager(m_claw, m_elevator, m_drive, m_trajectoryManager);

  private final XboxController m_driverStick = new XboxController(0);
  private final XboxController m_operatorStick = new XboxController(1);

  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Compressor m_compressor =
      new Compressor(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);

  private void logException(Exception e) {
    try {
      m_exceptionHappened = true;
      if (!RobotBase.isSimulation()) {
        FileWriter fileWriter = new FileWriter("/home/lvuser/exception_log.txt", true);
        PrintWriter printWriter = new PrintWriter(fileWriter);
        e.printStackTrace(printWriter);
        printWriter.close();
        fileWriter.close();
      }

      System.err.println(e);
    } catch (Exception ie) {
      throw new RuntimeException("Could not write to exception log file", ie);
    }
  }

  private void dashboardUpdateSubsystems() {
    m_elevator.dashboardUpdate();
    m_claw.dashboardUpdate();
    m_drive.dashboardUpdate();
    m_candleManager.dashboardUpdate();
  }

  /** Update subsystems. Called me when enabled. */
  private void updateSubsystems() {
    m_elevator.update();
    m_claw.update();
    m_drive.update();
  }

  /** Reset subsystems. Called me when initializing. */
  private void resetSubsystems() {
    m_elevator.reset();
    m_claw.reset();
    m_drive.reset();
    m_candleManager.reset();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    try {
      GreyDashClient.setAvailableAutoModes(
          AutoMode.Test, AutoMode.OneCone, AutoMode.PreloadAndCharge, AutoMode.NoAuto);
      GreyDashClient.availableGamePieces(GamePiece.Cone, GamePiece.Cube, GamePiece.None);

      this.resetSubsystems();
    } catch (Exception e) {
      logException(e);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    try {
      // Subsystems
      m_candleManager.update();
      if (isEnabled()) {
        updateSubsystems();
      }
      // GreyDash
      GreyDashClient.update();
      dashboardUpdateSubsystems();

      // Auto Selection
      m_autoManager.selectAuto(GreyDashClient.getAutoSelected());

      // CANdle
      if (!m_exceptionHappened
          || !isDisabled() && m_candleManager.getLightState() != LightState.GotIt) {
        m_candleManager.setLightWithGamePiece();
      }
    } catch (Exception e) {
      logException(e);
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    try {
      m_compressor.enableDigital();
      m_autoManager.init();
      m_autoRan = true;
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    try {
      m_autoManager.run();
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    try {
      m_compressor.enableDigital();
      m_claw.setWristState(WristState.Manual);
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {
      /////////////////////
      // DRIVER CONTROLS //
      /////////////////////
      final double xSpeed = -MathUtil.applyDeadband(m_driverStick.getRawAxis(1), 0.12);
      final double ySpeed = -MathUtil.applyDeadband(m_driverStick.getRawAxis(0), 0.12);

      double rot =
          -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverStick.getRawAxis(4), 0.09))
              * DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      if (m_elevator.getHeight() > 15.0) {
        rot *= 0.5;
      }

      Translation2d translation =
          new Translation2d(xSpeed, ySpeed).times(DriveInfo.MAX_VELOCITY_METERS_PER_SECOND);

      m_drive.driveInput(translation, rot, true);

      // Closed loop drive angle
      if (m_driverStick.getYButton()) {
        m_drive.setRotationControl(RotationControl.ClosedLoop);
        m_drive.setTargetRobotAngle(Drive.AnglePresets.TOWARDS_DS);
      } else if (m_driverStick.getBButton()) {
        m_drive.setRotationControl(RotationControl.ClosedLoop);
        m_drive.setTargetRobotAngle(Drive.AnglePresets.TOWARDS_HP);
      } else if (rot == 0.0) {
        if (m_driverStick.getYButtonReleased() || m_driverStick.getBButtonReleased()) {
          m_drive.setTargetRobotAngle(m_drive.getNormalizedGyroYaw());
        }
        m_drive.setRotationControl(RotationControl.ClosedLoop);
      } else {
        m_drive.setRotationControl(RotationControl.OpenLoop);
      }

      // Reset Drive
      if (m_driverStick.getStartButton()) {
        m_drive.reset();
      }

      // Score
      if (m_driverStick.getLeftBumper()) {
        m_claw.setIntakeState(IntakeState.Out);
      } else if (m_claw.getIntakeState() == IntakeState.Out) {
        m_claw.setIntakeState(IntakeState.Neutral);
        m_currentGamePiece = GamePiece.None;
        m_claw.setWristPreset(WristPreset.Stow);
        m_elevator.setHeight(Elevator.Presets.stow);
      }

      // Right Cone
      if (m_driverStick.getRightTriggerAxis() > 0.1) {
        m_elevator.setElevatorState(ElevatorState.ClosedLoop);
        m_elevator.setHeight(Elevator.Presets.floor);
        m_claw.setWristState(WristState.ClosedLoop);
        m_currentGamePiece = GamePiece.Cone;
        m_claw.setIntakeState(IntakeState.In);
        if (m_driverStick.getRightTriggerAxis() > 0.9) {
          m_claw.setWristPreset(WristPreset.Floor);
        } else {
          m_claw.setWristPreset(WristPreset.ConeRight);
        }
      }

      //////////
      // BOTH //
      //////////
      // Stow elevator/wrist
      if (m_driverStick.getLeftTriggerAxis() > 0.5 || m_operatorStick.getAButton()) {
        m_elevator.setHeight(Elevator.Presets.stow);
        m_claw.setWristPreset(Claw.WristPreset.Stow);
      }

      ////////////////////////
      // CO-DRIVER CONTROLS //
      ////////////////////////
      double operatorStickRightY = -MathUtil.applyDeadband(m_operatorStick.getRawAxis(5), 0.1);

      // Elevator height preset
      switch (m_operatorStick.getPOV()) {
        case 0:
          m_elevator.setHeight(Elevator.Presets.high);
          m_claw.setWristPreset(Claw.WristPreset.High);
          break;
        case 90:
          m_elevator.setHeight(Elevator.Presets.mid);
          m_claw.setWristPreset(Claw.WristPreset.Mid);
          break;
        case 180:
          m_elevator.setHeight(Elevator.Presets.floor);
          m_claw.setWristPreset(Claw.WristPreset.Floor);
          break;
        case 270:
          m_elevator.setHeight(Elevator.Presets.hp);
          m_claw.setWristPreset(Claw.WristPreset.HP);
          break;
        default:
          break;
      }

      // Manual Elevator
      if (operatorStickRightY != 0.0) {
        m_elevator.setElevatorState(ElevatorState.Manual);
        m_elevator.setElevatorOutput(operatorStickRightY);
      } else if (m_elevator.getElevatorState() == ElevatorState.Manual) {
        m_elevator.setHeight(m_elevator.getHeight());
        m_elevator.setElevatorState(ElevatorState.ClosedLoop);
      } else {
        m_elevator.setElevatorState(ElevatorState.ClosedLoop);
      }

      // Intake
      if (m_operatorStick.getRightTriggerAxis() > 0.5) {
        m_currentGamePiece = GamePiece.Cone;
        m_claw.setIntakeState(IntakeState.In);
      } else if (m_operatorStick.getLeftTriggerAxis() > 0.5) {
        m_currentGamePiece = GamePiece.Cube;
        m_claw.setIntakeState(IntakeState.In);
      } else if (m_claw.getIntakeState() != IntakeState.Out
          && m_claw.getIntakeState() != IntakeState.Neutral
          && m_driverStick.getRightTriggerAxis() < 0.1) {
        m_claw.setIntakeState(IntakeState.Hold);
      }

      // Got it!
      if (m_claw.getIntakeState() == IntakeState.In && m_claw.checkForGamePiece()) {
        if (!m_gotIt) {
          m_gotItStartTime = Conversions.Time.getMsecTime();
          m_gotIt = true;
        }
        m_candleManager.setLightState(LightState.GotIt);

        m_claw.setWristPreset(WristPreset.Stow);
        if (Conversions.Time.getMsecTime() - m_gotItStartTime > GOT_IT_DELAY_MSEC) {
          m_elevator.setHeight(Elevator.Presets.stow);
          m_gotIt = false;
        }
      }

      // Manually Control Wrist
      double wristJoystickInput = -MathUtil.applyDeadband(m_operatorStick.getLeftY(), 0.12) * 0.25;
      if (wristJoystickInput != 0.0 && m_driverStick.getRightTriggerAxis() <= 0.1) {
        m_claw.setWristState(WristState.Manual);
        m_claw.setWristMotorOutput(wristJoystickInput);
      } else {
        m_claw.setWristState(WristState.ClosedLoop);
      }
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    try {
      m_compressor.disable();
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    try {
      if (!m_autoRan) {
        m_currentGamePiece = GreyDashClient.selectedGamePiece();
      }
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    try {
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    try {
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    try {
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    try {
    } catch (Exception e) {
      logException(e);
    }
  }
}
