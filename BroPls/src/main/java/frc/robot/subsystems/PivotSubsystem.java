package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private static final int pivotMotorID = 45;
    private final SparkMax pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder;
    private final ProfiledPIDController m_controller;
    private final ProfiledPIDController m_holdController; // Separate controller for holding position
    
    // Manual control constants - REDUCED FOR SMOOTHER OPERATION
    private static final double MANUAL_SPEED = 0.4; // Reduced from 1.0 to make movement gentler
    private static final double JOYSTICK_DEADBAND = 0.1;

    // Position constants
    private static final double ZERO_POSITION = 0.0;
    private static final double GROUND_INTAKE_POSITION = 3.0; // Keeping this one
    private static final double TROUGH_POSITION = 22.0; // New position
    private static final double L2L3_POSITION = 25.0; // New position
    private static final double L4_POSITION = 28.5; // New position
    
    private double m_goalAngle = 0.0; // Start at zero position instead of preset
    private boolean isMovementEnabled = false;
    private boolean isManualControl = false;
    private boolean holdPosition = true; // START in hold position mode to prevent drift at startup
    private double lastVelocity = 0;
    private boolean hasBeenReset = false; // Flag to track if we've been reset during this session

    public PivotSubsystem() {
        m_encoder = pivotMotor.getEncoder();
        
        // Don't auto-zero encoder on startup to maintain position across restarts
        if (!hasBeenReset) {
            // Set initial goal to current position, not a preset
            m_goalAngle = m_encoder.getPosition();
        }
        
        SmartDashboard.putNumber("Initial Encoder Position", m_encoder.getPosition());
       
        // Movement constraints - SLOWER FOR SMOOTHER OPERATION
        TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(5, 0.5); // Reduced velocity and acceleration
        
        // Main controller for movement to target - INCREASED PID VALUES FOR MORE POWER
        m_controller = new ProfiledPIDController(0.1, 0.03, 0.08, m_constraints);
        m_controller.reset(m_encoder.getPosition());
        
        // Holding controller with even gentler values
        m_holdController = new ProfiledPIDController(0.008, 0.002, 0.003, m_constraints);
        m_holdController.reset(m_encoder.getPosition());
        m_holdController.setGoal(m_encoder.getPosition()); // Initialize with current position
        
        // Wider tolerance for hold controller
        m_holdController.setTolerance(0.2);
    }

    public void setGoalAngle(double angle) {
        m_goalAngle = angle;
        isMovementEnabled = true;
        isManualControl = false;
        holdPosition = false;
        
        // Reset both controllers with the new goal
        m_controller.reset(m_encoder.getPosition());
        m_holdController.reset(m_encoder.getPosition());
    }

    public void stopMovement() {
        // Don't disable position holding when stopping, just stop active movement
        isMovementEnabled = false;
        isManualControl = false;
        // Don't change holdPosition - keep it as is to maintain position
        pivotMotor.set(0);
    }
    
    /**
     * Manually control the pivot with a joystick
     * @param speed The speed to set (-1 to 1)
     */
    public void manualControl(double speed) {
        // Apply deadband
        if (Math.abs(speed) < JOYSTICK_DEADBAND) {
            // If we were in manual control but now joystick is released, switch to hold position
            if (isManualControl) {
                isManualControl = false;
                holdPosition = true;
                
                // First stop the motor immediately to prevent continued motion
                pivotMotor.set(0);
                
                // Update goal angle to current position for holding
                m_goalAngle = m_encoder.getPosition();
                
                // Reset hold controller at current position
                m_holdController.reset(m_encoder.getPosition());
                m_holdController.setGoal(m_goalAngle);
                
                // Enabled for position holding
                isMovementEnabled = true;
            }
            return;
        }
        
        // Joystick is being actively used, enable manual control
        
        // Invert speed for desired direction (joystick up = pivot down)
        speed = -speed;
        
        // Apply non-linear curve for finer control at low speeds
        speed = Math.signum(speed) * Math.pow(Math.abs(speed), 1.5);
        
        // Apply manual speed multiplier
        speed *= MANUAL_SPEED;
        
        // Set manual control flag and disable other modes
        isManualControl = true;
        isMovementEnabled = false;
        holdPosition = false;
        
        // Apply speed to motor with additional smoothing
        pivotMotor.set(speed);
        
        // Update goal angle to current position
        m_goalAngle = m_encoder.getPosition();
    }
    
    /**
     * Move to zero position
     */
    public void moveToZero() {
        System.out.println("Moving pivot to zero position");
        setGoalAngle(ZERO_POSITION);
    }

    @Override
    public void periodic() {
        // Track velocity for use in determining when arm has stopped
        lastVelocity = m_encoder.getVelocity();
        
        // If this is the first periodic call after startup, initialize hold position
        if (!isManualControl && !isMovementEnabled && holdPosition) {
            // Reset hold controller to maintain current position
            m_holdController.setGoal(m_encoder.getPosition());
        }
        
        // Handle position holding
        if (holdPosition) {
            // Hold position logic - using gentler PID values
            SmartDashboard.putBoolean("Pivot Manual Control", false);
            SmartDashboard.putBoolean("Pivot Hold Position", true);
            
            // Use hold controller to maintain position
            double currentPosition = m_encoder.getPosition();
            double speed = m_holdController.calculate(currentPosition);
            
            // Apply a more restrictive speed limit for holding to prevent oscillations
            speed = Math.max(-0.07, Math.min(0.07, speed)); // Reduced from 0.1
            
            // Only apply motor power if the error is above a minimum threshold
            double error = Math.abs(currentPosition - m_goalAngle);
            if (error > 0.1) { // Increased threshold to reduce small adjustments
                pivotMotor.set(speed);
            } else {
                // Very small error, apply minimal holding power
                double holdingPower = 0.005 * Math.signum(speed); // Reduced from 0.01
                pivotMotor.set(holdingPower);
            }
        } else if (isManualControl) {
            // Manual control is handled in the manualControl method
            SmartDashboard.putBoolean("Pivot Manual Control", true);
            SmartDashboard.putBoolean("Pivot Hold Position", false);
        } else if (isMovementEnabled) {
            // Automatic movement logic
            SmartDashboard.putBoolean("Pivot Manual Control", false);
            SmartDashboard.putBoolean("Pivot Hold Position", false);
            
            m_controller.setGoal(m_goalAngle);
       
            // Compute motor output using PID
            double currentPosition = m_encoder.getPosition();
            double speed = m_controller.calculate(currentPosition);
       
            // Limit speed to prevent high-torque issues - INCREASED FOR MORE POWER
            speed = Math.max(-0.6, Math.min(0.6, speed)); // Increased from 0.3
       
            // Apply speed to motor
            pivotMotor.set(speed);
       
            // Stop motor if it's already at the target
            if (isPivotAtGoal()) {
                // Don't stop the motor completely - switch to hold position mode
                holdPosition = true;
                isMovementEnabled = false;
                m_holdController.reset(currentPosition);
                m_holdController.setGoal(m_goalAngle);
            }
        } else {
            // We're not in any active control mode, maintain position
            holdPosition = true;
            m_goalAngle = m_encoder.getPosition();
            m_holdController.setGoal(m_goalAngle);
        }
        
        // Always update these values
        SmartDashboard.putNumber("Pivot Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Pivot Goal Position", m_goalAngle);
        SmartDashboard.putNumber("Pivot Speed", pivotMotor.get());
        SmartDashboard.putNumber("Pivot Velocity", m_encoder.getVelocity());
        SmartDashboard.putBoolean("Pivot At Goal", isPivotAtGoal());
    }

    // Preset position methods
    public void goToGroundIntake() {
        setGoalAngle(GROUND_INTAKE_POSITION);
    }

    public void goToTrough() {
        setGoalAngle(TROUGH_POSITION);
    }

    public void goToL2L3() {
        setGoalAngle(L2L3_POSITION);
    }

    public void goToL4() {
        setGoalAngle(L4_POSITION);
    }
    
    /**
     * Reset the encoder position to zero
     */
    public void resetEncoder() {
        pivotMotor.getEncoder().setPosition(0);
        m_goalAngle = 0;
        m_controller.reset(0);
        m_holdController.reset(0);
        m_holdController.setGoal(0);
        holdPosition = true;  // Enable holding at zero position
        isMovementEnabled = false;
        isManualControl = false;
        hasBeenReset = true;  // Mark that we've been manually reset
        SmartDashboard.putString("Pivot Status", "Encoder Reset");
    }

    public boolean isPivotAtGoal() {
        double currentPosition = m_encoder.getPosition();
        double error = Math.abs(currentPosition - m_goalAngle);
       
        // Debug values
        SmartDashboard.putNumber("Pivot Error", error);
        SmartDashboard.putNumber("Current Position", currentPosition);
        SmartDashboard.putNumber("Goal Position", m_goalAngle);
       
        return error < 0.5; // Increased tolerance
    }

    /**
     * Create a command to reset the encoder position to zero
     * @return A command that resets the encoder
     */
    public Command createResetEncoderCommand() {
        return runOnce(this::resetEncoder);
    }
    
    /**
     * Creates a command to move to a preset position and stay there,
     * even after the button is released.
     * @param targetPosition The target position to move to
     * @return Command for preset movement that maintains position after release
     */
    private Command createPresetPositionCommand(double targetPosition) {
        return Commands.sequence(
            // First run the command to move to the target position
            Commands.runOnce(() -> {
                System.out.println("Moving pivot to position: " + targetPosition);
                setGoalAngle(targetPosition);
                isMovementEnabled = true;
                holdPosition = false;
                
                // Reset the controller for a fresh start
                m_controller.reset(m_encoder.getPosition());
                m_controller.setGoal(targetPosition);
            }),
            
            // Apply higher power initially to overcome inertia
            Commands.runOnce(() -> {
                // Get direction to target
                double currentPosition = m_encoder.getPosition();
                double direction = Math.signum(targetPosition - currentPosition);
                
                // Apply a strong initial pulse to overcome any static friction
                pivotMotor.set(direction * 0.5);
            }),
            
            // Brief delay to let the initial pulse take effect
            Commands.waitSeconds(0.1),
            
            // Run until we reach the target
            Commands.run(() -> {
                // Run automatic movement via the periodic method
                isMovementEnabled = true;
                holdPosition = false;
                
                // Periodically log progress for debugging
                if (Math.random() < 0.05) { // Log about 5% of the time
                    double currentPosition = m_encoder.getPosition();
                    System.out.println("Pivot moving... current: " + currentPosition + ", target: " + targetPosition);
                }
            })
            .until(() -> isPivotAtGoal()),
            
            // Hold at the target position
            Commands.runOnce(() -> {
                System.out.println("Pivot at target, holding position at: " + m_encoder.getPosition());
                holdPosition = true;
                isMovementEnabled = false;
                m_holdController.setGoal(targetPosition);
                m_goalAngle = targetPosition;
            }),
            
            // Wait indefinitely until button is released
            Commands.waitUntil(() -> false)
        ).finallyDo((interrupted) -> {
            // When button is released or command interrupted
            System.out.println("Button released, maintaining current position");
            
            // Get current position to hold
            double currentPosition = m_encoder.getPosition();
            
            // Create and schedule a command to hold at current position
            Commands.runOnce(() -> {
                // Stay at current position
                holdPosition = true;
                isMovementEnabled = false;
                m_goalAngle = currentPosition;
                m_holdController.reset(currentPosition);
                m_holdController.setGoal(currentPosition);
            }).schedule();
        });
    }

    /**
     * Creates a command that moves to zero position
     */
    public Command createMoveToZeroCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("Moving pivot to zero");
                setGoalAngle(ZERO_POSITION);
            }),
            Commands.run(() -> {
                // The periodic method handles the actual movement
            })
            .until(() -> Math.abs(m_encoder.getPosition()) < 0.2),
            Commands.runOnce(() -> {
                // Hold at zero
                holdPosition = true;
                isMovementEnabled = false;
                m_goalAngle = ZERO_POSITION;
                m_holdController.setGoal(ZERO_POSITION);
            })
        );
    }
    
    // Command methods for the new presets
    public Command createGroundIntakeCommand() {
        return createPresetPositionCommand(GROUND_INTAKE_POSITION);
    }
   
    public Command createTroughCommand() {
        return createPresetPositionCommand(TROUGH_POSITION);
    }
   
    public Command createL2L3Command() {
        return createPresetPositionCommand(L2L3_POSITION);
    }
   
    public Command createL4Command() {
        return createPresetPositionCommand(L4_POSITION);
    }
}