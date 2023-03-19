// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.shared.RobotInfo.*;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.List;

import frc.robot.AutoManager.AutoMode;
import frc.robot.AutoManager.AutoSide;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.shared.LimelightHelpers;
import frc.robot.subsystems.CANdleManager;
import frc.robot.subsystems.CANdleManager.LightState;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.RotationControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPreset;
import frc.robot.subsystems.Wrist.WristState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  @Getter private static GamePiece m_preloadGamePiece = GamePiece.Cone;

  @Getter private static boolean m_exceptionHappened = false;

  @Getter private static Alliance m_calculatedAlliance;

  private static boolean m_autoRan = false;

  private final Elevator m_elevator = new Elevator();
  private final Wrist m_wrist = new Wrist();
  private final Claw m_claw = new Claw();
  private final Drive m_drive = new Drive();
  private final CANdleManager m_candleManager = new CANdleManager();
  private final AutoManager m_autoManager = new AutoManager(m_claw, m_elevator, m_drive, m_wrist);
  private final Superstructure m_superStructure = new Superstructure(m_wrist, m_elevator);

  private final XboxController m_driverStick = new XboxController(0);
  private final XboxController m_operatorStick = new XboxController(1);

  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Compressor m_compressor =
      new Compressor(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);

  private final List<AutoMode> m_availableAutoModes =
      Arrays.asList(
          AutoMode.PreloadPickupCharge,
          AutoMode.Test,
          AutoMode.PreloadAndCharge,
          AutoMode.CenterPreloadAndCharge,
          AutoMode.PreloadPickupScoreCharge,
          AutoMode.NoAuto);
  private int m_selectedMode = 0;
  private AutoSide m_selectedAutoSide = AutoSide.Left;

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

      if (isSimulation()) {
        throw e;
      }
    } catch (Exception ie) {
      throw new RuntimeException("Could not write to exception log file", ie);
    }
  }

  private void dashboardUpdateSubsystems() {
    m_elevator.dashboardUpdate();
    m_wrist.dashboardUpdate();
    m_claw.dashboardUpdate();
    m_drive.dashboardUpdate();
    m_candleManager.dashboardUpdate();
    m_superStructure.dashboardUpdate();
  }

  /** Update subsystems. Called me when enabled. */
  private void updateSubsystems() {
    m_elevator.update();
    m_wrist.update();
    m_claw.update();
    m_drive.update();
    m_superStructure.update();
  }

  /** Reset subsystems. Called me when initializing. */
  private void resetSubsystems() {
    m_elevator.reset();
    m_wrist.reset();
    m_claw.reset();
    m_drive.reset();
    m_candleManager.reset();
    m_superStructure.reset();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    try {
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
      dashboardUpdateSubsystems();

      AutoSide side = m_selectedAutoSide;
      switch (DriverStation.getAlliance()) {
        case Blue:
          if (side == AutoSide.Left) {
            m_calculatedAlliance = Alliance.Blue;
          } else {
            m_calculatedAlliance = Alliance.Red;
          }
          break;
        case Red:
          if (side == AutoSide.Left) {
            m_calculatedAlliance = Alliance.Blue;
          } else {
            m_calculatedAlliance = Alliance.Red;
          }
          break;
        case Invalid:
          m_calculatedAlliance = Alliance.Blue;
          break;
      }

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
      LimelightHelpers.setPipelineIndex("", 1);
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
      LimelightHelpers.setPipelineIndex("", 0);
      m_compressor.enableDigital();
      m_wrist.setState(WristState.Manual);
      m_drive.setTargetRobotAngle(m_drive.getNormalizedGyroYaw());
      m_drive.disableBrakeMode();
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
      } else if (m_driverStick.getAButton()) {
        m_drive.setTargetRobotAngle(
            DriverStation.getAlliance() == Alliance.Red
                ? Drive.AnglePresets.TOWARDS_WS_RED
                : Drive.AnglePresets.TOWARDS_WS_BLUE);
      } else if (m_driverStick.getXButton()) {
        m_drive.setTargetRobotAngle(
            DriverStation.getAlliance() == Alliance.Red
                ? Drive.AnglePresets.TOWARDS_WSR_RED
                : Drive.AnglePresets.TOWARDS_WSR_BLUE);
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
        if (m_claw.isHasGamePiece() && (m_wrist.getPreset() == WristPreset.Stow)) {
          m_wrist.setPreset(WristPreset.ConeRight);
        }
        m_claw.setIntakeState(IntakeState.Out);
      } else if (m_claw.getIntakeState() == IntakeState.Out) {
        m_claw.setIntakeState(IntakeState.Neutral);
        m_currentGamePiece = GamePiece.None;
        m_wrist.setPreset(WristPreset.Stow);
        m_elevator.setPreset(Elevator.Preset.Stow);
      }

      // Right Cone
      if (m_driverStick.getRightTriggerAxis() > 0.1) {
        m_elevator.setElevatorState(ElevatorState.ClosedLoop);
        m_elevator.setPreset(Elevator.Preset.Floor);
        m_wrist.setState(WristState.ClosedLoop);
        m_currentGamePiece = GamePiece.Cone;
        m_claw.setIntakeState(IntakeState.In);
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
        m_elevator.setPreset(Elevator.Preset.Stow);
        m_wrist.setPreset(WristPreset.Stow);
      }

      ////////////////////////
      // CO-DRIVER CONTROLS //
      ////////////////////////
      double operatorStickRightY = -MathUtil.applyDeadband(m_operatorStick.getRawAxis(5), 0.1);

      if (m_operatorStick.getAButton()) {
        m_elevator.setPreset(Elevator.Preset.MiniHp);
        m_wrist.setPreset(WristPreset.MiniHp);
        m_currentGamePiece = GamePiece.None;
      }

      // Elevator height preset
      switch (m_operatorStick.getPOV()) {
        case 0:
          m_elevator.setPreset(Elevator.Preset.High);
          m_wrist.setPreset(WristPreset.High);
          break;
        case 90:
          m_elevator.setPreset(Elevator.Preset.Mid);
          m_wrist.setPreset(WristPreset.Mid);
          break;
        case 180:
          // If we have a game piece, go to hybrid, otherwise go to floor
          if (m_claw.isHasGamePiece()) {
            m_elevator.setPreset(Elevator.Preset.Hybrid);
            m_wrist.setPreset(WristPreset.Hybrid);
          } else {
            m_elevator.setPreset(Elevator.Preset.Floor);
            m_wrist.setPreset(WristPreset.Floor);
          }
          break;
        case 270:
          m_elevator.setPreset(Elevator.Preset.Hp);
          m_wrist.setPreset(WristPreset.HP);
          m_currentGamePiece = GamePiece.None;
          break;
        default:
          break;
      }

      // Manual Elevator
      if (operatorStickRightY != 0.0) {
        m_elevator.setElevatorState(ElevatorState.Manual);
        m_elevator.setElevatorOutput(operatorStickRightY);
        m_superStructure.joystickPressed(true);
      } else {
        m_elevator.setElevatorState(ElevatorState.ClosedLoop);
        m_superStructure.joystickPressed(false);
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
      if (m_claw.getIntakeState() == IntakeState.In && m_claw.isHasGamePiece()) {
        m_candleManager.setLightState(LightState.GotIt);

        m_wrist.setPreset(WristPreset.Stow);
        m_elevator.setPreset(Elevator.Preset.Stow);
      }

      // Manually Control Wrist
      double wristJoystickInput = -MathUtil.applyDeadband(m_operatorStick.getLeftY(), 0.12) * 0.25;

      if (wristJoystickInput != 0.0 && m_driverStick.getRightTriggerAxis() <= 0.1) {
        m_wrist.setState(WristState.Manual);
        m_wrist.setMotorOutput(wristJoystickInput);
      } else {
        m_wrist.setState(WristState.ClosedLoop);
      }

      if (m_operatorStick.getStartButton()) {
        m_claw.reset();
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
      if (m_operatorStick.getYButtonPressed()) {
        m_selectedMode += 1;
      }
      if (m_operatorStick.getAButtonPressed()) {
        m_selectedMode -= 1;
      }
      if (m_selectedMode >= m_availableAutoModes.size()) {
        m_selectedMode = 0;
      }
      if (m_selectedMode < 0) {
        m_selectedMode = m_availableAutoModes.size() - 1;
      }

      if (m_operatorStick.getXButtonPressed()) {
        m_selectedAutoSide = AutoSide.Left;
      }
      if (m_operatorStick.getBButtonPressed()) {
        m_selectedAutoSide = AutoSide.Right;
      }

      if (m_operatorStick.getLeftBumperPressed()) {
        m_preloadGamePiece = GamePiece.Cone;
      }
      if (m_operatorStick.getRightBumperPressed()) {
        m_preloadGamePiece = GamePiece.Cube;
      }

      SmartDashboard.putString("DB/String 0", m_availableAutoModes.get(m_selectedMode).toString());
      SmartDashboard.putString("DB/String 1", m_selectedAutoSide.toString());
      SmartDashboard.putString("DB/String 2", m_preloadGamePiece.toString());

      m_autoManager.selectAuto(m_availableAutoModes.get(m_selectedMode));

      if (m_driverStick.getAButton()) {
        m_drive.enableBrakeMode();
      } else {
        m_drive.disableBrakeMode();
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
