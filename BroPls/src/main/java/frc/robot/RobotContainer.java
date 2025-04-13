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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);

  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final Elevator elevator = new Elevator();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() ,
                                                                () -> driverXbox.getLeftX() )
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Set up elevator manual control with left joystick
    elevator.setDefaultCommand(
        Commands.run(() -> {
            double joystickValue = -operatorXbox.getLeftY();
            elevator.setManualSpeed(joystickValue * 0.4); // Increased from 0.3 to 0.4 for more responsive manual control
        }, elevator)
    );

    // Set up pivot manual control with right joystick
    m_pivotSubsystem.setDefaultCommand(
        Commands.run(() -> {
            double joystickValue = operatorXbox.getRightY();
            m_pivotSubsystem.manualControl(joystickValue);
        }, m_pivotSubsystem)
    );
    // Reef alignment
		driverXbox.povRight().onTrue(new AlignToReefTagRelative(true, drivebase).withTimeout(3));
		driverXbox.povLeft().onTrue(new AlignToReefTagRelative(false, drivebase).withTimeout(3));

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(6,
                                                                     1,
                                                                     .5,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(6,
                                                                     1,
                                                                     .5,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

     driverXbox.b().whileTrue(
         drivebase.driveToPose(
             new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                             );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.rightBumper().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue(Commands.none());
    }
    
    // Elevator presets mapped to ABXY buttons with corrected positions
    operatorXbox.a().whileTrue(Commands.sequence(
        Commands.runOnce(() -> System.out.println("*** A PRESSED - ACTIVATING TROUGH PRESET ***")),
        elevator.createTroughCommand()
    ));
    
    operatorXbox.b().whileTrue(Commands.sequence(
        Commands.runOnce(() -> System.out.println("*** B PRESSED - ACTIVATING L4 PRESET ***")),
        elevator.createL4Command()
    ));
    
    operatorXbox.x().whileTrue(Commands.sequence(
        Commands.runOnce(() -> System.out.println("*** X PRESSED - ACTIVATING L2 PRESET ***")),
        elevator.createL2Command()
    ));
    
    operatorXbox.y().whileTrue(Commands.sequence(
        Commands.runOnce(() -> System.out.println("*** Y PRESSED - ACTIVATING L3 PRESET ***")),
        elevator.createL3Command()
    ));

    // Map pivot presets to D-pad with debug prints
    operatorXbox.povDown().whileTrue(Commands.sequence(
        Commands.runOnce(() -> System.out.println("*** D-PAD DOWN - ACTIVATING PIVOT TROUGH PRESET ***")),
        m_pivotSubsystem.createTroughCommand()
    ));
    
    operatorXbox.povLeft().whileTrue(Commands.sequence(
        Commands.runOnce(() -> System.out.println("*** D-PAD LEFT - ACTIVATING PIVOT L2/L3 PRESET ***")),
        m_pivotSubsystem.createL2L3Command()
    ));
    
    operatorXbox.povUp().whileTrue(Commands.sequence(
        Commands.runOnce(() -> System.out.println("*** D-PAD UP - ACTIVATING PIVOT L2/L3 PRESET ***")),
        m_pivotSubsystem.createL2L3Command()
    ));
    
    operatorXbox.povRight().whileTrue(Commands.sequence(
        Commands.runOnce(() -> System.out.println("*** D-PAD RIGHT - ACTIVATING PIVOT L4 PRESET ***")),
        m_pivotSubsystem.createL4Command()
    ));

    // Reset encoders and emergency stop
    operatorXbox.leftBumper().onTrue(m_pivotSubsystem.createResetEncoderCommand());
    operatorXbox.rightBumper().onTrue(elevator.createZeroEncoderCommand());

    // Shooter subsystem commands - left trigger for intake, right trigger for shooting
    Trigger operatorLeftTrigger = new Trigger(() -> operatorXbox.getLeftTriggerAxis() > 0.1);
    Trigger operatorRightTrigger = new Trigger(() -> operatorXbox.getRightTriggerAxis() > 0.1);
    
    operatorLeftTrigger.whileTrue(
        Commands.runEnd(
            // While active, run intake
            () -> shooterSubsystem.intake(),
            // When finished, stop motors
            () -> shooterSubsystem.stop(),
            shooterSubsystem
        )
    );
    
    operatorRightTrigger.whileTrue(
        Commands.runEnd(
            // While active, run shooter
            () -> shooterSubsystem.shoot(),
            // When finished, stop motors
            () -> shooterSubsystem.stop(),
            shooterSubsystem
        )
    );

    // Emergency stop button combination (press both bumpers together)
    Trigger bothBumpers = operatorXbox.rightBumper().and(operatorXbox.leftBumper());
    bothBumpers.onTrue(Commands.runOnce(() -> {
        // Emergency stop function
        System.out.println("EMERGENCY STOP ACTIVATED");
        elevator.stopAndHold(); // This will stop the elevator and hold position
        m_pivotSubsystem.stopMovement(); // Stop the pivot subsystem
        shooterSubsystem.stop(); // Stop the shooter
    }));
    
    // Driver slow mode - left trigger reduces max speed to 1 foot per second
    Trigger driverLeftTrigger = new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.1);
    driverLeftTrigger.onTrue(Commands.runOnce(() -> {
        System.out.println("SLOW MODE ACTIVATED - 1ft/s");
        // Use the scaleTranslation method to reduce speed to roughly 1ft/s
        driveAngularVelocity.scaleTranslation(0.1);  // Reduce to 10% of normal speed
    }));
    driverLeftTrigger.onFalse(Commands.runOnce(() -> {
        System.out.println("SLOW MODE DEACTIVATED - returning to normal speed");
        // Restore to 80% speed (the default we set earlier)
        driveAngularVelocity.scaleTranslation(0.8);
    }));
  }
  
  /**
   * Creates a fixed command sequence that coordinates pivot and elevator movement
   * with improved reliability and no oscillation.
   * 
   * @param pivotPosition The pivot position to move to
   * @param elevatorPosition The elevator position to move to
   * @return A coordinated command sequence
   */
  private Command createCoordinatedMovementCommand(double pivotPosition, double elevatorPosition) {
    return Commands.sequence(
        // First move pivot to desired position - use direct command instead of the command factory
        Commands.runOnce(() -> {
            System.out.println("Starting coordinated movement - Moving pivot to: " + pivotPosition);
            m_pivotSubsystem.setGoalAngle(pivotPosition);
        }, m_pivotSubsystem),
        
        // Wait until pivot reaches position or timeout
        Commands.run(() -> {
            // The pivot's periodic method handles the actual movement
        }, m_pivotSubsystem)
        .until(() -> m_pivotSubsystem.isPivotAtGoal())
        .withTimeout(3), // Timeout for safety
        
        // Once pivot is at position, start the elevator
        Commands.runOnce(() -> {
            System.out.println("Pivot positioned, now moving elevator to: " + elevatorPosition);
            // Direct command to elevator to move to position
        }),
        
        // Now run the elevator's command (not as a sequence member, but as a parallel command)
        Commands.parallel(
            // Hold the pivot in position
            Commands.run(() -> {
                // Pivot will hold position thanks to its periodic method
            }, m_pivotSubsystem),
            
            // Run the elevator to position
            elevator.createPresetHeightCommand(elevatorPosition)
        ),
        
        // The elevator command contains the waitUntil(false) that keeps everything active
        // until the button is released, and its finallyDo handles returning to zero
        
        // Wait indefinitely until button is released
        Commands.waitUntil(() -> false)
    ).finallyDo((interrupted) -> {
        // When button is released, reverse the sequence
        System.out.println("Button released, returning to zero in sequence");
        
        // Create and schedule the return sequence
        Commands.sequence(
            // First move elevator down to zero
            Commands.runOnce(() -> {
                System.out.println("Moving elevator to zero");
                elevator.setPosition(0); // Direct command to move to zero
            }, elevator),
            
            // Wait until elevator is near zero
            Commands.run(() -> {})
            .until(() -> Math.abs(elevator.getCurrentPosition()) < 0.5)
            .withTimeout(3), // Safety timeout
            
            // Then move pivot back to zero
            Commands.runOnce(() -> {
                System.out.println("Moving pivot to zero");
                m_pivotSubsystem.setGoalAngle(0);
            }, m_pivotSubsystem)
        ).schedule();
    });
  }


  /**
   * Creates the Ground Intake coordinated position command
   */
  private Command createCoordinatedGroundIntakeCommand() {
    return createCoordinatedMovementCommand(
        3.0, // Ground intake pivot position
        12.0 // Algae1 elevator position
    );
  }

  /**
   * Creates the Algae1 coordinated position command
   */
  private Command createCoordinatedAlgae1Command() {
    return createCoordinatedMovementCommand(
        0.5, // Algae1 pivot position
        12.0 // Algae1 elevator position
    );
  }

  /**
   * Creates the Algae2 coordinated position command
   */
  private Command createCoordinatedAlgae2Command() {
    return createCoordinatedMovementCommand(
        0.7, // Algae2 pivot position
        22.0 // Algae2 elevator position
    );
  }

  /**
   * Creates the Barge Shoot coordinated position command
   */
  private Command createCoordinatedBargeCommand() {
    return createCoordinatedMovementCommand(
        0.9, // Barge pivot position
        30.0 // Barge elevator position
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.driveCommand(()->-1, ()->0, ()->0).withDeadline(new WaitCommand(1));
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}