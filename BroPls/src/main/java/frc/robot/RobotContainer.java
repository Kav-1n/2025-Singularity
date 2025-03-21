// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.AlignToTagPosition;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightConstants;
import swervelib.SwerveInputStream;

import java.io.File;

public class RobotContainer {
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);
  // public final CommandXboxController limelightXbox = new CommandXboxController(2);

  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final Elevator elevator = new Elevator();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final MultiTagLimelight m_multiTagLimelight = new MultiTagLimelight();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private boolean m_frontSelected = true;
  private int m_horizontalPosition = 1; // 0 = left, 1 = middle, 2 = right
  private int m_selectedLevel = 2;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
          driverXbox::getLeftY,
          driverXbox::getLeftX)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
      .robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> -driverXbox.getLeftY(),
          () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
          () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(0));

  public RobotContainer() {
    configureBindings();

    elevator.setDefaultCommand(Commands.run(() -> {
      double joystickValue = -operatorXbox.getLeftY();
      elevator.setManualSpeed(joystickValue * 0.4);
    }, elevator));
    // Set up elevator manual control with left joystick
    elevator.setDefaultCommand(
        Commands.run(() -> {
            double joystickValue = -operatorXbox.getLeftY();
            elevator.setManualSpeed(joystickValue * 0.3);
        }, elevator)
    );

    m_pivotSubsystem.setDefaultCommand(Commands.run(() -> {
      double joystickValue = operatorXbox.getRightY();
      m_pivotSubsystem.manualControl(joystickValue);
    }, m_pivotSubsystem));

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  public void periodic() {
    double leftY = limelightXbox.getLeftY();
    if (Math.abs(leftY) > 0.5) {
      m_frontSelected = leftY < 0;
      updateFrontBackDisplay();
    }

    double rightX = limelightXbox.getRightX();
    if (Math.abs(rightX) > 0.5) {
      m_horizontalPosition = (rightX < -0.5) ? 0 : (rightX > 0.5) ? 2 : 1;
      updateHorizontalPositionDisplay();
    }
  }

  private void updateFrontBackDisplay() {
    SmartDashboard.putBoolean("Limelight Front/Back", m_frontSelected);
  }

  private void updateHorizontalPositionDisplay() {
    String position = switch (m_horizontalPosition) {
      case 0 -> "LEFT";
      case 1 -> "MIDDLE";
      case 2 -> "RIGHT";
      default -> "UNKNOWN";
    };
    SmartDashboard.putString("Limelight Horizontal Position", position);
  }

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    if (RobotBase.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));

      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(6, 1, 0.5, new Constraints(5, 2)),
          new ProfiledPIDController(6, 1, 0.5,
              new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));

      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(
          Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
              () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      driverXbox.b().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
    }

    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    } else {
      driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue(Commands.none());
    }
    
    // Coordinated preset position commands - pivot moves first, then elevator
    // When button is released, elevator returns to zero first, then pivot
    operatorXbox.a().whileTrue(createCoordinatedGroundIntakeCommand());
    operatorXbox.b().whileTrue(createCoordinatedAlgae1Command());
    operatorXbox.x().whileTrue(createCoordinatedAlgae2Command());
    operatorXbox.y().whileTrue(createCoordinatedBargeCommand());

    // Keep the D-pad for individual elevator control (if desired)
    operatorXbox.povUp().whileTrue(elevator.createAlgae1Command());
    operatorXbox.povRight().whileTrue(elevator.createAlgae2Command());
    operatorXbox.povLeft().whileTrue(elevator.createBargeCommand());
    operatorXbox.povDown().onTrue(elevator.createMoveToZeroCommand());

    operatorXbox.leftBumper().onTrue(m_pivotSubsystem.createResetEncoderCommand());
    operatorXbox.rightBumper().onTrue(elevator.createZeroEncoderCommand());

    // Shooter controls
    Trigger operatorLeftTrigger = new Trigger(() -> operatorXbox.getLeftTriggerAxis() > 0.1);
    Trigger operatorRightTrigger = new Trigger(() -> operatorXbox.getRightTriggerAxis() > 0.1);

    operatorLeftTrigger.whileTrue(Commands.runEnd(shooterSubsystem::intake, shooterSubsystem::stop, shooterSubsystem));
    operatorRightTrigger.whileTrue(Commands.runEnd(shooterSubsystem::shoot, shooterSubsystem::stop, shooterSubsystem));

    Trigger bothBumpers = operatorXbox.rightBumper().and(operatorXbox.leftBumper());
    bothBumpers.onTrue(Commands.runOnce(() -> {
      System.out.println("EMERGENCY STOP ACTIVATED");
      elevator.stopAndHold();
      m_pivotSubsystem.stopMovement();
      shooterSubsystem.stop();
    }));

    Trigger driverLeftTrigger = new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.1);
    driverLeftTrigger.onTrue(Commands.runOnce(() -> {
      System.out.println("SLOW MODE ACTIVATED - 1ft/s");
      driveAngularVelocity.scaleTranslation(0.1);
    }));
    driverLeftTrigger.onFalse(Commands.runOnce(() -> {
      System.out.println("SLOW MODE DEACTIVATED");
      driveAngularVelocity.scaleTranslation(0.8);
    }));

    // Limelight Bindings
    limelightXbox.a().onTrue(Commands.runOnce(() -> {
      m_selectedLevel = 1;
      SmartDashboard.putNumber("Limelight Selected Level", m_selectedLevel);
    }));

    limelightXbox.x().onTrue(Commands.runOnce(() -> {
      m_selectedLevel = 2;
      SmartDashboard.putNumber("Limelight Selected Level", m_selectedLevel);
    }));

    limelightXbox.y().onTrue(Commands.runOnce(() -> {
      m_selectedLevel = 3;
      SmartDashboard.putNumber("Limelight Selected Level", m_selectedLevel);
    }));

    limelightXbox.b().onTrue(Commands.runOnce(() -> {
      m_selectedLevel = 4;
      SmartDashboard.putNumber("Limelight Selected Level", m_selectedLevel);
    }));

    Trigger limelightAlignLeftTrigger = new Trigger(() ->
        limelightXbox.leftBumper().getAsBoolean() && m_horizontalPosition != -1);
    limelightAlignLeftTrigger.onTrue(Commands.runOnce(() -> {
      int tagId = determineTagId(true);
      m_multiTagLimelight.setTarget(tagId, true, m_selectedLevel);
      new AlignToTagPosition(m_multiTagLimelight, drivebase).schedule();
    }));

    Trigger limelightAlignRightTrigger = new Trigger(limelightXbox.rightBumper().and(() -> m_horizontalPosition != -1));
    limelightAlignRightTrigger.onTrue(Commands.runOnce(() -> {
      int tagId = determineTagId(false);
      m_multiTagLimelight.setTarget(tagId, false, m_selectedLevel);
      new AlignToTagPosition(m_multiTagLimelight, drivebase).schedule();
    }));
  }

  private int determineTagId(boolean isLeft) {
    return switch (m_horizontalPosition) {
      case 0 -> isLeft ? 3 : 2;
      case 1 -> 2;
      case 2 -> isLeft ? 4 : 3;
      default -> 2;
    };
  }

  public Command getAutonomousCommand() {
    return drivebase.driveCommand(() -> -1, () -> 0, () -> 0).withDeadline(new WaitCommand(1));
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
