// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.shared.RobotInfo.*;

import java.io.FileWriter;
import java.io.PrintWriter;

import frc.robot.greydash.GreyDashClient;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.CANdleManager;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static String m_autoSelected = kDefaultAuto;

  private final Elevator m_elevator = new Elevator();
  private final Claw m_claw = new Claw();
  private final Drive m_drive = new Drive();
  private final CANdleManager m_candle = new CANdleManager();

  private final XboxController m_driverStick = new XboxController(0);
  private final XboxController m_operatorStick = new XboxController(1);

  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Setter @Getter private GamePiece m_currentGamePiece;

  private final Compressor m_compressor =
      new Compressor(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);

  private void logException(Exception e) {
    try {
      FileWriter fileWriter = new FileWriter("/home/lvuser/exception_log.txt", true);
      PrintWriter printWriter = new PrintWriter(fileWriter);
      e.printStackTrace(printWriter);
      printWriter.close();
      fileWriter.close();

      System.err.println(e);
    } catch (Exception ie) {
      throw new RuntimeException("Could not write to exception log file", ie);
    }
  }

  /** Update subsystems. Called me when enabled. */
  private void updateSubsystems() {
    m_elevator.update();
    m_claw.update();
    m_drive.update();
    m_candle.update();
  }

  /** Reset subsystems. Called me when initializing. */
  private void resetSubsystems() {
    m_elevator.reset();
    m_claw.reset();
    m_drive.reset();
    m_candle.reset();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    GreyDashClient.setAvailableAutoModes(kDefaultAuto, kCustomAuto);

    this.resetSubsystems();
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
      GreyDashClient.update();
      if (this.isEnabled()) {
        this.updateSubsystems();
      }
      m_candle.setLightWithGamePiece(m_currentGamePiece);
      SmartDashboard.putNumber("Elevator Height", m_elevator.getHeight());
      SmartDashboard.putNumber("Elevator Position", m_elevator.getPosition());
      SmartDashboard.putBoolean("Elevator Bottom Hall", m_elevator.getBottomHall());
      SmartDashboard.putBoolean("Elevator Top Hall", m_elevator.getTopHall());
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
    m_autoSelected = GreyDashClient.getAutoSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    m_compressor.enableDigital();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    try {
      this.updateSubsystems();
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_compressor.enableDigital();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {
      /////////////////////
      // DRIVER CONTROLS //
      /////////////////////
      final double xSpeed = -MathUtil.applyDeadband(m_driverStick.getRawAxis(1), 0.09);
      final double ySpeed = -MathUtil.applyDeadband(m_driverStick.getRawAxis(0), 0.09);

      final double rot =
          -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverStick.getRawAxis(4), 0.09))
              * DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

      SmartDashboard.putNumber("drive/swerve/inputs/xspeed", xSpeed);
      SmartDashboard.putNumber("drive/swerve/inputs/yspeed", ySpeed);
      SmartDashboard.putNumber("drive/swerve/inputs/rot", rot);

      Translation2d translation =
          new Translation2d(xSpeed, ySpeed).times(DriveInfo.MAX_VELOCITY_METERS_PER_SECOND);

      m_drive.driveInput(translation, rot, true);

      // Closed loop drive angle
      if (m_driverStick.getRightBumper()) {
        m_drive.setRotationControl(RotationControl.ClosedLoop);
        m_drive.setTargetRobotAngle(Drive.AnglePresets.TOWARDS_DS);
      } else if (m_driverStick.getRightTriggerAxis() > 0.5) {
        m_drive.setRotationControl(RotationControl.ClosedLoop);
        m_drive.setTargetRobotAngle(Drive.AnglePresets.TOWARDS_HP);
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
        m_claw.setCurrentGamePiece(GamePiece.Cone);
        m_claw.setIntakeState(IntakeState.In);
      } else if (m_operatorStick.getLeftTriggerAxis() > 0.5) {
        m_claw.setCurrentGamePiece(GamePiece.Cube);
        m_claw.setIntakeState(IntakeState.In);
      } else if (m_claw.getIntakeState() != IntakeState.Out
          && m_claw.getIntakeState() != IntakeState.Neutral) {
        m_claw.setIntakeState(IntakeState.Hold);
      }

      // Manually Control Wrist
      double wristJoystickInput = -MathUtil.applyDeadband(m_operatorStick.getLeftY(), 0.12) * 0.25;
      if (wristJoystickInput != 0.0) {
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
    m_compressor.disable();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    try {
    } catch (Exception e) {
      logException(e);
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    try {
    } catch (Exception e) {
      logException(e);
    }
  }
}
