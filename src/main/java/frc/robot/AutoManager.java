package frc.robot;

import frc.robot.auto.commands.DriveTrajectoryCommand;
import frc.robot.auto.commands.ElevatorPresetCommand;
import frc.robot.auto.commands.IntakeCommand;
import frc.robot.auto.commands.PathPlannerTrajectoryCommand;
import frc.robot.auto.commands.SetDrivePositionCommand;
import frc.robot.auto.commands.TrajectoryManager;
import frc.robot.auto.commands.WristAngleCommand;
import frc.robot.auto.commands.util.ConcurrentCommand;
import frc.robot.auto.commands.util.SequentialCommand;
import frc.robot.auto.commands.util.WaitCommand;
import frc.robot.shared.AutoCommand;
import frc.robot.shared.Constants.GamePiece;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.IntakeState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoManager {
  private final Claw m_claw;
  private final Elevator m_elevator;
  private final Drive m_drive;
  private final TrajectoryManager m_trajectoryManager;

  private GamePiece m_preload;

  private AutoCommand m_currentMode;

  public enum AutoMode {
    Test,
    OneCone,
    PreloadAndCharge,
    NoAuto
  }

  private final AutoCommand test;
  private final AutoCommand oneCone;
  private final AutoCommand preloadAndCharge;
  private final AutoCommand noAuto = new SequentialCommand();

  public AutoManager(
      Claw claw, Elevator elevator, Drive drive, TrajectoryManager trajectoryManager) {
    m_claw = claw;
    m_elevator = elevator;
    m_drive = drive;
    m_trajectoryManager = trajectoryManager;

    test = new SequentialCommand(
        new SetDrivePositionCommand(
            drive,
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)),
            Rotation2d.fromDegrees(180.0)),
        new DriveTrajectoryCommand(m_drive, m_trajectoryManager.getTrajectoryA()));

    oneCone = new SequentialCommand(
        new SetDrivePositionCommand(
            drive,
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)),
            Rotation2d.fromDegrees(180.0)),
        new IntakeCommand(claw, IntakeState.In, GamePiece.Cone, 100),
        new ElevatorPresetCommand(elevator, Elevator.Presets.high, 4000),
        new WaitCommand(500),
        new WristAngleCommand(claw, Claw.ConePresets.high, 2000),
        new IntakeCommand(claw, IntakeState.Out, GamePiece.Cone, 1500),
        new WaitCommand(500),
        new ConcurrentCommand(
            new ElevatorPresetCommand(elevator, Elevator.Presets.stow, 1000),
            new WristAngleCommand(claw, Claw.ConePresets.stow, 2000)),
        new DriveTrajectoryCommand(m_drive, m_trajectoryManager.getTrajectoryA()));

    preloadAndCharge = new SequentialCommand(
        new PathPlannerTrajectoryCommand(
            m_drive, "PreloadAndCharge", new PathConstraints(4, 3), true));
  }

  public void run() {
    m_currentMode.run();
  }

  public void init() {
    m_currentMode.init();
  }

  public void selectAuto(AutoMode mode) {
    switch (mode) {
      case Test:
        m_currentMode = test;
        break;
      case OneCone:
        m_currentMode = oneCone;
        break;
      case PreloadAndCharge:
        m_currentMode = preloadAndCharge;
        break;
      case NoAuto:
        m_currentMode = noAuto;
        break;
    }
  }

  public void selectPreload(GamePiece preload) {
    m_preload = preload;
  }
}
