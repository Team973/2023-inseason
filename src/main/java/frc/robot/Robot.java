// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.shared.RobotInfo.*;

import java.io.FileWriter;
import java.io.PrintWriter;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ExtensionState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.GamePiece;
import frc.robot.subsystems.Intake.IntakeState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake m_intake = new Intake();
  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm();
  private final Drive m_drive = new Drive();

  private final XboxController m_driverStick = new XboxController(0);
  private final XboxController m_operatorStick = new XboxController(1);

  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Compressor m_compressor =
      new Compressor(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);

  private void logException(Exception e) {
    try {
      FileWriter fileWriter = new FileWriter("/home/lvuser/exception_log.txt", true);
      PrintWriter printWriter = new PrintWriter(fileWriter);
      e.printStackTrace(printWriter);
      printWriter.close();
      fileWriter.close();
    } catch (Exception ie) {
      throw new RuntimeException("could not write to exception log file", ie);
    }
  }

  /** Update subsystems. Called me when enabled. */
  private void updateSubsystems() {
    m_exampleSubsystem.update();
    m_intake.update();
    m_elevator.update();
    m_arm.update();
    m_drive.update();
  }

  /** Reset subsystems. Called me when initializing. */
  private void resetSubsystems() {
    m_exampleSubsystem.reset();
    m_intake.reset();
    m_elevator.reset();
    m_arm.reset();
    m_drive.reset();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

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
      if (this.isEnabled()) {
        this.updateSubsystems();
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_compressor.enableDigital();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    try {
      this.updateSubsystems();
      switch (m_autoSelected) {
        case kCustomAuto:
          // Put custom auto code here
          break;
        case kDefaultAuto:
        default:
          // Put default auto code here
          break;
      }
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
      final double xSpeed = -MathUtil.applyDeadband(m_driverStick.getRawAxis(1), 0.09);
      final double ySpeed = -MathUtil.applyDeadband(m_driverStick.getRawAxis(0), 0.09);

      final double rot =
          -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverStick.getRawAxis(4), 0.09))
              * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

      SmartDashboard.putNumber("drive/swerve/inputs/xspeed", xSpeed);
      SmartDashboard.putNumber("drive/swerve/inputs/yspeed", ySpeed);
      SmartDashboard.putNumber("drive/swerve/inputs/rot", rot);

      Translation2d translation =
          new Translation2d(xSpeed, ySpeed).times(DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);

      m_drive.driveInput(translation, rot, true);
    } catch (Exception e) {
      logException(e);
    }

    // Arm extension
    if (m_operatorStick.getLeftBumper()) {
      m_arm.setExtensionState(ExtensionState.RETRACTED);
    } else if (m_operatorStick.getRightBumper()) {
      m_arm.setExtensionState(ExtensionState.EXTENDED);
    }

    // Elevator height preset
    switch (m_operatorStick.getPOV()) {
      case 0:
        m_elevator.setHighPreset();
        break;
      case 90:
        m_elevator.setMidPreset();
        break;
      case 180:
        m_elevator.setFloorPreset();
        break;
      case 270:
        m_elevator.setHpPreset();
        break;
    }

    double operatorStickRightY = MathUtil.applyDeadband(m_operatorStick.getRawAxis(0), 0.1);
    double operatorStickRightX = MathUtil.applyDeadband(m_operatorStick.getRawAxis(1), 0.1);

    // Manual Elevator
    if (operatorStickRightY != 0.0) {
      m_elevator.setElevatorState(ElevatorState.Manual);
      m_elevator.setElevatorOutput(operatorStickRightY);
    } else {
      m_elevator.setElevatorState(ElevatorState.ClosedLoop);
    }

    // Intake
    if (operatorStickRightX < 0.0) {
      m_intake.setIntakeState(IntakeState.In);
    } else if (operatorStickRightX > 0.0) {
      m_intake.setIntakeState(IntakeState.Out);
    }

    // Select Game Piece
    if (m_operatorStick.getBButton()) {
      m_intake.setCurrentGamePiece(GamePiece.Cube);
    } else if (m_operatorStick.getXButton()) {
      m_intake.setCurrentGamePiece(GamePiece.Cone);
    }

    // Set Wrist Angle
    m_arm.setWristTargetAngle(MathUtil.applyDeadband(m_operatorStick.getRawAxis(1), 0.09));
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
