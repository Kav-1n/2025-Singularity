// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, zController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = -1;
  private String limelightName = "limelight";

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
    // Create properly tuned PID controllers
    zController = new PIDController(Constants.Z_REEF_ALIGNMENT_P, 0.25, 0.5);  // Forward/back
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.25, 0.5);  // Left/right 
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0.25, 0.5);  // Rotation
    
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    // Set rotation goal (should be zero to face tag directly)
    rotController.setSetpoint(0.0);  // Face the tag
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    // Set forward/back distance goal
    zController.setSetpoint(Constants.Z_SETPOINT_REEF_ALIGNMENT);  // Distance from tag
    zController.setTolerance(Constants.Z_TOLERANCE_REEF_ALIGNMENT);

    // Set left/right position goal
    double xSetpoint = isRightScore ? Constants.X_SETPOINT_REEF_ALIGNMENT : -Constants.X_SETPOINT_REEF_ALIGNMENT;
    xController.setSetpoint(xSetpoint);  // Offset left or right from center
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    // Lock onto a tag if we see one
    if (LimelightHelpers.getTV(limelightName)) {
      tagID = LimelightHelpers.getFiducialID(limelightName);
      System.out.println("Alignment initialized - Tracking tag ID: " + tagID);
    } else {
      System.out.println("WARNING: No AprilTag visible at alignment start");
    }
  }

  @Override
  public void execute() {
    boolean canSeeTag = LimelightHelpers.getTV(limelightName);
    
    if (canSeeTag) {
      // Reset the "don't see tag" timer since we can see a tag
      this.dontSeeTagTimer.reset();
      
      // Get robot's position relative to the tag
      double[] positions = LimelightHelpers.getBotPose_TargetSpace(limelightName);
      
      // Log current position for debugging
      System.out.println("Current Position - X: " + positions[0] + 
                        ", Y: " + positions[1] + 
                        ", Z: " + positions[2] + 
                        ", Yaw: " + positions[5]);
      
      // Calculate drive commands using PID controllers
      double xSpeed = xController.calculate(positions[0]);  // Left/right 
      double zSpeed = zController.calculate(positions[2]);  // Forward/back 
      double rotSpeed = rotController.calculate(positions[5]);  // Rotation (yaw)
      
      // Apply a deadband to prevent tiny oscillations
      if (Math.abs(xSpeed) < 0.02) xSpeed = 0;
      if (Math.abs(zSpeed) < 0.02) zSpeed = 0;
      if (Math.abs(rotSpeed) < 0.01) rotSpeed = 0;
      
      // Limit maximum speeds
      double maxLinearSpeed = 0.2;
      double maxRotSpeed = 0.1;
      
      // Clamp values
      xSpeed = Math.max(-maxLinearSpeed, Math.min(maxLinearSpeed, xSpeed));
      zSpeed = Math.max(-maxLinearSpeed, Math.min(maxLinearSpeed, zSpeed));
      rotSpeed = Math.max(-maxRotSpeed, Math.min(maxRotSpeed, rotSpeed));
      
      // Log control outputs
      System.out.println("Control outputs - X: " + xSpeed + 
                        ", Z: " + zSpeed + 
                        ", Rot: " + rotSpeed);
      
      // Drive the robot
      drivebase.drive(new Translation2d(zSpeed, xSpeed), rotSpeed, false);
      
      // Check if we've reached the target position
      boolean atTarget = xController.atSetpoint() && 
                         zController.atSetpoint() && 
                         rotController.atSetpoint();
      
      // Report status
      System.out.println("At target position: " + atTarget);
      
      // If not at target, reset the timer
      if (!atTarget) {
        stopTimer.reset();
      }
    } else {
      // Lost sight of the tag
      System.out.println("Tag not visible");
      drivebase.drive(new Translation2d(), 0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Alignment command ended");
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Finish if we've lost the tag for too long or have been at the target position
    boolean lostTagTooLong = dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME);
    boolean atPositionLongEnough = stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
    
    return lostTagTooLong || atPositionLongEnough;
  }
}