package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class PivotSubsystem extends SubsystemBase {


    private static final int pivotMotorID = 45;
    private final SparkMax pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder;
    private final ProfiledPIDController m_controller;
    
    // Manual control constants
    private static final double MANUAL_SPEED = 10; // Speed multiplier for manual control
    private static final double JOYSTICK_DEADBAND = 0.1;


    // Renamed constants to match elevator subsystem
    private static final double GROUND_INTAKE_POSITION = 3.0; // Placeholder angle
    private static final double ALGAE1_POSITION = 0.5; // Placeholder angle
    private static final double ALGAE2_POSITION = 0.7; // Placeholder angle
    private static final double BARGE_SHOOT_POSITION = 0.9; // Placeholder angle (was algae3)
    
    private double m_goalAngle = GROUND_INTAKE_POSITION;
    private boolean isMovementEnabled = false;
    private boolean isManualControl = false;


    public PivotSubsystem() {
        m_encoder = pivotMotor.getEncoder();
        pivotMotor.getEncoder().setPosition(0); // Zeroing encoder


        m_goalAngle = m_encoder.getPosition(); // Set initial goal to current position
        SmartDashboard.putNumber("Initial Encoder Position", m_encoder.getPosition());
       
        TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(10, 1);
        m_controller = new ProfiledPIDController(0.01, 0, 0.005, m_constraints);
        m_controller.reset(m_encoder.getPosition());
    }


    public void setGoalAngle(double angle) {
        m_goalAngle = angle;
        isMovementEnabled = true;
        isManualControl = false;  // Disable manual control when setting a goal angle
    }


    public void stopMovement() {
        isMovementEnabled = false;
        isManualControl = false;
        pivotMotor.set(0);
    }
    
    /**
     * Manually control the pivot with a joystick
     * @param speed The speed to set (-1 to 1)
     */
    public void manualControl(double speed) {
        // Apply deadband
        if (Math.abs(speed) < JOYSTICK_DEADBAND) {
            speed = 0;
            if (isManualControl) {
                pivotMotor.set(0);
            }
            return;
        }
        
        // Invert speed for desired direction (joystick up = pivot down)
        
        
        speed = -speed;
        
        // Apply manual speed multiplier
        speed *= MANUAL_SPEED;
        
        // Set manual control flag and disable automatic movement
        isManualControl = true;
        isMovementEnabled = false;
        
        // Apply speed to motor
        pivotMotor.set(speed);
        
        // Update goal angle to current position (for smoother transition to automatic control)
        m_goalAngle = m_encoder.getPosition();
    }


    @Override
    public void periodic() {
        if (isManualControl) {
            // Manual control is handled in the manualControl method
            SmartDashboard.putBoolean("Pivot Manual Control", true);
        } else if (!isMovementEnabled) {
            pivotMotor.set(0);  // Stop the motor if movement is not enabled
            SmartDashboard.putBoolean("Pivot Manual Control", false);
        } else {
            // Automatic movement logic
            m_controller.setGoal(m_goalAngle);
       
            // Compute motor output using PID
            double currentPosition = m_encoder.getPosition();
            double speed = m_controller.calculate(currentPosition, m_goalAngle);
       
            // Limit speed to prevent high-torque issues
            speed = Math.max(-0.5, Math.min(0.5, speed));
       
            // Apply speed to motor
            pivotMotor.set(speed);
       
            // Stop motor if it's already at the target
            if (isPivotAtGoal()) {
                pivotMotor.stopMotor();
                isMovementEnabled = false;
            }
            
            SmartDashboard.putBoolean("Pivot Manual Control", false);
        }
        
        // Always update these values
        SmartDashboard.putNumber("Pivot Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Pivot Goal Position", m_goalAngle);
        SmartDashboard.putNumber("Pivot Speed", pivotMotor.get());
        SmartDashboard.putBoolean("Pivot At Goal", isPivotAtGoal());
    }


    // Renamed methods to match elevator subsystem
    public void goToGroundIntake() {
        setGoalAngle(GROUND_INTAKE_POSITION);
    }


    public void goToAlgae1() {
        setGoalAngle(ALGAE1_POSITION);
    }


    public void goToAlgae2() {
        setGoalAngle(ALGAE2_POSITION);
    }


    public void goToBargeShoot() {
        setGoalAngle(BARGE_SHOOT_POSITION);
    }
    
    /**
     * Reset the encoder position to zero
     */
    public void resetEncoder() {
        pivotMotor.getEncoder().setPosition(0);
        m_goalAngle = 0;
        m_controller.reset(0);
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
     * Create a command to move the pivot with the right joystick
     * @param operatorXbox The operator's CommandXboxController
     * @return A command that moves the pivot based on joystick input
     */
    public Command createJoystickCommand(CommandXboxController operatorXbox) {
        return run(() -> {
            // Get joystick value (up = positive, down = negative)
            double speed = operatorXbox.getRightY() * MANUAL_SPEED;
            manualControl(speed);
        });
    }


    /**
     * Create a command to reset the encoder position to zero
     * @return A command that resets the encoder
     */
    public Command createResetEncoderCommand() {
        return runOnce(this::resetEncoder);
    }


    // Updated command methods with proper naming to match elevator subsystem
    public Command createGroundIntakeCommand() {
        return this.runOnce(() -> {
            setGoalAngle(GROUND_INTAKE_POSITION);
            isMovementEnabled = true;
        })
        .andThen(this.run(() -> {}))
        .until(() -> {
            double error = Math.abs(m_encoder.getPosition() - m_goalAngle);
            boolean isMoving = Math.abs(m_encoder.getVelocity()) > 0.01;
            boolean atGoal = error < 0.03;
            return atGoal && !isMoving;
        });
    }
   
    public Command createAlgae1Command() {
        return this.runOnce(() -> {
            setGoalAngle(ALGAE1_POSITION);
            isMovementEnabled = true;
        })
        .andThen(this.run(() -> {}))
        .until(() -> {
            double error = Math.abs(m_encoder.getPosition() - m_goalAngle);
            boolean isMoving = Math.abs(m_encoder.getVelocity()) > 0.01;
            boolean atGoal = error < 0.03;
            return atGoal && !isMoving;
        });
    }
   
    public Command createAlgae2Command() {
        return this.runOnce(() -> {
            setGoalAngle(ALGAE2_POSITION);
            isMovementEnabled = true;
        })
        .andThen(this.run(() -> {}))
        .until(() -> {
            double error = Math.abs(m_encoder.getPosition() - m_goalAngle);
            boolean isMoving = Math.abs(m_encoder.getVelocity()) > 0.01;
            boolean atGoal = error < 0.03;
            return atGoal && !isMoving;
        });
    }
   
    public Command createBargeShootCommand() {
        return this.runOnce(() -> {
            setGoalAngle(BARGE_SHOOT_POSITION);
            isMovementEnabled = true;
        })
        .andThen(this.run(() -> {}))
        .until(() -> {
            double error = Math.abs(m_encoder.getPosition() - m_goalAngle);
            boolean isMoving = Math.abs(m_encoder.getVelocity()) > 0.01;
            boolean atGoal = error < 0.03;
            return atGoal && !isMoving;
        });
    }
}



