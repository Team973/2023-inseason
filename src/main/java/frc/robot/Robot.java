// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.shared.RobotInfo.*;

import frc.robot.devices.GreyPigeon;
import frc.robot.shared.Conversions.MathHelpers;
import frc.robot.shared.CrashTracker;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.RotationControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.GlobalState;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;
import frc.robot.subsystems.Wrist.WristState;
import frc.robot.subsystems.candle.CANdleManager;
import frc.robot.subsystems.candle.CANdleManager.LightState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.experimental.Accessors;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Accessors(prefix = "m_")
public class Robot extends TimedRobot {
  @Getter private static GamePiece m_preloadGamePiece = GamePiece.Cone;

  @Getter private static Alliance m_calculatedAlliance;

  private final GreyPigeon m_pigeon = new GreyPigeon();
  private final Elevator m_elevator = new Elevator();
  private final Wrist m_wrist = new Wrist(m_pigeon);
  private final Claw m_claw = new Claw();
  private final Drive m_drive = new Drive(m_pigeon);
  private final CANdleManager m_candleManager = new CANdleManager();
  private final Superstructure m_superstructure = new Superstructure(m_elevator, m_wrist, m_claw);
  private final AutoManager m_autoManager = new AutoManager(m_drive, m_superstructure);
  private final XboxController m_driverStick = new XboxController(0);
  private final XboxController m_operatorStick = new XboxController(1);

  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private void dashboardUpdateSubsystems() {
    m_elevator.dashboardUpdate();
    m_wrist.dashboardUpdate();
    m_claw.dashboardUpdate();
    m_drive.dashboardUpdate();
    m_candleManager.dashboardUpdate();
    m_superstructure.dashboardUpdate();
  }

  private void debugDashboardUpdateSubsystems() {
    m_elevator.debugDashboardUpdate();
    m_wrist.debugDashboardUpdate();
    m_claw.debugDashboardUpdate();
    m_drive.debugDashboardUpdate();
    m_candleManager.debugDashboardUpdate();
    m_superstructure.debugDashboardUpdate();
  }

  /** Update subsystems. Called me when enabled. */
  private void updateSubsystems() {
    m_elevator.update();
    m_wrist.update();
    m_claw.update();
    m_drive.update();
    m_superstructure.update();
  }

  /** Reset subsystems. Called me when initializing. */
  private void resetSubsystems() {
    m_elevator.reset();
    m_wrist.reset();
    m_claw.reset();
    m_drive.reset();
    m_candleManager.reset();
    m_superstructure.reset();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    try {
      CrashTracker.logRobotInit();
      this.resetSubsystems();
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
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

      dashboardUpdateSubsystems();

      if (!DriverStation.isFMSAttached()) {
        debugDashboardUpdateSubsystems();
      } else {
        CrashTracker.logFMSData();
      }

      m_calculatedAlliance = DriverStation.getAlliance();

      // CANdle
      if (!CrashTracker.isExceptionHappened()
          || !isDisabled() && m_candleManager.getLightState() != LightState.GotIt) {
        m_candleManager.setLightWithGamePiece();
      }
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
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
      CrashTracker.logAutoInit();
      m_autoManager.init();
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    try {
      m_autoManager.run();
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    try {
      CrashTracker.logTeleopInit();
      m_drive.setTargetRobotAngle(m_drive.getPigeon().getNormalizedYaw());
      m_drive.disableBrakeMode();
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {
      /////////////////////
      // DRIVER CONTROLS //
      /////////////////////
      final double xSpeed =
          m_driverStick.getBackButton()
              ? 0.18
              : MathHelpers.signSquare(-MathUtil.applyDeadband(m_driverStick.getRawAxis(1), 0.12));
      final double ySpeed =
          MathHelpers.signSquare(-MathUtil.applyDeadband(m_driverStick.getRawAxis(0), 0.12));

      double rot =
          -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverStick.getRawAxis(4), 0.09))
              * DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      if (m_elevator.getHeight() > 15.0) {
        rot *= 0.5;
      }

      Translation2d translation =
          new Translation2d(xSpeed, ySpeed).times(DriveInfo.MAX_VELOCITY_METERS_PER_SECOND);

      if (m_driverStick.getXButton()) {
        translation = translation.times(m_elevator.getMinimumToCurrentHeightRatio());
      }

      m_drive.driveInput(translation, rot);

      // Closed loop drive angle
      if (m_driverStick.getYButton()) {
        m_drive.setRotationControl(RotationControl.ClosedLoop);
        m_drive.setTargetRobotAngle(Drive.AnglePresets.TOWARDS_DS);
      } else if (m_driverStick.getBButton()) {
        m_drive.setRotationControl(RotationControl.ClosedLoop);
        m_drive.setTargetRobotAngle(Drive.AnglePresets.TOWARDS_HP);
      } else if (rot == 0.0) {
        if (m_driverStick.getYButtonReleased() || m_driverStick.getBButtonReleased()) {
          m_drive.setTargetRobotAngle(m_drive.getPigeon().getNormalizedYaw());
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
        if (m_claw.isHasGamePiece()
            && (m_superstructure.getCurrentGlobalState() == GlobalState.Stow
                || m_superstructure.getCurrentGlobalState() == GlobalState.Toss)) {
          m_superstructure.setDesiredGlobalState(GlobalState.Toss);
        } else {
          m_superstructure.setDesiredGlobalState(GlobalState.Score);
        }
      } else if (m_superstructure.getCurrentGlobalState() == GlobalState.Score) {
        m_superstructure.setDesiredGlobalState(GlobalState.PostScore);
      }

      // Right Cone
      if (m_driverStick.getRightTriggerAxis() > 0.1) {
        m_elevator.setElevatorState(ElevatorState.ClosedLoop);
        m_elevator.setPreset(Elevator.Preset.Floor);
        m_wrist.setState(WristState.ClosedLoop);
        Superstructure.setCurrentGamePiece(GamePiece.Cone);
        m_superstructure.setDesiredIntakeState(IntakeState.In);
        if (m_driverStick.getRightTriggerAxis() > 0.9) {
          m_wrist.setPreset(WristPreset.Floor);
        } else {
          m_wrist.setPreset(WristPreset.ConeRight);
        }
      }

      //////////
      // BOTH //
      //////////
      // Stow elevator/wrist
      if (m_driverStick.getLeftTriggerAxis() > 0.5) {
        m_superstructure.setDesiredGlobalState(GlobalState.Stow);
      }

      ////////////////////////
      // CO-DRIVER CONTROLS //
      ////////////////////////
      double operatorStickRightY = -MathUtil.applyDeadband(m_operatorStick.getRawAxis(5), 0.12);

      // Elevator height preset
      switch (m_operatorStick.getPOV()) {
        case 0:
          m_superstructure.setDesiredGlobalState(GlobalState.ScoreHigh);
          break;
        case 90:
          m_superstructure.setDesiredGlobalState(GlobalState.ScoreMid);
          break;
        case 180:
          // If we have a game piece, go to hybrid, otherwise go to floor
          if (m_claw.isHasGamePiece()) {
            m_superstructure.setDesiredGlobalState(GlobalState.ScoreLow);
          } else {
            m_superstructure.setDesiredGlobalState(GlobalState.LoadFloor);
          }
          break;
        case 270:
          m_superstructure.setDesiredGlobalState(GlobalState.LoadHp);
          Superstructure.setCurrentGamePiece(GamePiece.None);
          break;
        default:
          break;
      }

      if (m_operatorStick.getAButton()) {
        m_superstructure.setDesiredGlobalState(GlobalState.Stow);
      }

      // Manual Elevator
      if (operatorStickRightY != 0.0) {
        m_elevator.setElevatorState(ElevatorState.Manual);
        m_elevator.setElevatorOutput(operatorStickRightY);
        m_superstructure.setDesiredGlobalState(GlobalState.Manual);
      } else {
        m_elevator.setElevatorState(ElevatorState.ClosedLoop);
      }

      // Intake
      if (m_operatorStick.getRightTriggerAxis() > 0.5) {
        Superstructure.setCurrentGamePiece(GamePiece.Cone);
        m_superstructure.setDesiredIntakeState(IntakeState.In);
      } else if (m_operatorStick.getLeftTriggerAxis() > 0.5) {
        Superstructure.setCurrentGamePiece(GamePiece.Cube);
        m_superstructure.setDesiredIntakeState(IntakeState.In);
      } else if (m_superstructure.getDesiredIntakeState() == IntakeState.In
          && m_driverStick.getRightTriggerAxis() < 0.1) {
        m_superstructure.setDesiredIntakeState(IntakeState.Hold);
      }

      // Got it!
      if (m_claw.getIntakeState() == IntakeState.In && m_claw.isHasGamePiece()) {
        m_candleManager.setLightState(LightState.GotIt);
        m_superstructure.setDesiredGlobalState(GlobalState.Stow);
      }

      // Manually Control Wrist
      double wristJoystickInput = -MathUtil.applyDeadband(m_operatorStick.getLeftY(), 0.12) * 0.25;
      if (wristJoystickInput != 0.0 && m_driverStick.getRightTriggerAxis() <= 0.1) {
        m_wrist.setState(WristState.Manual);
        m_superstructure.setDesiredGlobalState(GlobalState.Manual);
        m_wrist.setMotorOutput(wristJoystickInput);
      } else {
        m_wrist.setState(WristState.ClosedLoop);
      }

      if (m_operatorStick.getStartButton()) {
        m_claw.reset();
      }
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    try {
      CrashTracker.logDisabledInit();
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    try {
      if (m_operatorStick.getYButtonPressed()) {
        m_autoManager.increment();
      }
      if (m_operatorStick.getAButtonPressed()) {
        m_autoManager.decrement();
      }

      SmartDashboard.putString("DB/String 0", m_autoManager.getSelectedMode().toString());

    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    try {
      CrashTracker.logTestInit();
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    try {
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    try {
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    try {
    } catch (Exception e) {
      CrashTracker.logThrowableCrash(e);
    }
  }
}
