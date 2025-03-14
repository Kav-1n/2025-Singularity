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

    private static final int pivotMotorID = 45; // Replace '1' with the correct motor ID
    private final SparkMax pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder;
    private final ProfiledPIDController m_controller;
    private static final double deltaTime = 0.02;

    private static final double groundIntake = 3; // Placeholder angle
    private static final double algae1 = 0.5; // Placeholder angle
    private static final double algae2 = 0.7; // Placeholder angle
    private static final double algae3 = 0.9; // Placeholder angle
    private double m_goalAngle = groundIntake;
    private boolean isMovementEnabled = false;

    public PivotSubsystem() {
        m_encoder = pivotMotor.getEncoder();

        pivotMotor.getEncoder().setPosition(0); // Zeroing encoder

        m_goalAngle = m_encoder.getPosition(); // Set initial goal to current position
        SmartDashboard.putNumber("Initial Encoder Position", m_encoder.getPosition()); // Ensure encoder reads angle properly
        
        TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(3, 1);
        m_controller = new ProfiledPIDController(0.05, 0, 0.0, m_constraints);
        m_controller.reset(m_encoder.getPosition());
    }

    public void setGoalAngle(double angle) {
        m_goalAngle = angle;
        isMovementEnabled = true;  // Enable movement when goal is set
    }

    public void stopMovement() {
        isMovementEnabled = false;
        pivotMotor.set(0);
    }

    @Override
    public void periodic() {
        if (!isMovementEnabled) {
            pivotMotor.set(0);  // Stop the motor if movement is not enabled
        } else {
            // Add the rest of the movement logic here if movement is enabled
            m_controller.setGoal(m_goalAngle);
        
            // Compute motor output using PID
            double speed = m_controller.calculate(m_encoder.getPosition());
        
            // Limit speed to prevent high-torque issues
            speed = Math.max(-0.5, Math.min(0.5, speed));
        
            // Apply speed to motor
            pivotMotor.set(speed);
        
            // Stop motor if it's already at the target
            if (isPivotAtGoal()) {
                pivotMotor.stopMotor();
                isMovementEnabled = false;
            }
        
            SmartDashboard.putNumber("Pivot Position", m_encoder.getPosition());
            SmartDashboard.putNumber("Pivot Goal Position", m_goalAngle);
            SmartDashboard.putNumber("Pivot Speed", speed);
            SmartDashboard.putBoolean("Pivot At Goal", isPivotAtGoal());
        }
    }

    public void goToGroundIntake() {
        setGoalAngle(PivotSubsystem.groundIntake);
    }

    public void goToAlgae1() {
        setGoalAngle(PivotSubsystem.algae1);
    }

    public void goToAlgae2() {
        setGoalAngle(PivotSubsystem.algae2);
    }

    public void goToAlgae3() {
        setGoalAngle(PivotSubsystem.algae3);
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

    public Command moveToGroundIntake() {
        return this.runOnce(() -> {
            setGoalAngle(groundIntake);
            isMovementEnabled = true;
        })
        .andThen(this.run(() -> {}))
        .until(() -> {
            // Only end the command once we've started moving AND reached the goal
            double error = Math.abs(m_encoder.getPosition() - m_goalAngle);
            boolean isMoving = Math.abs(m_encoder.getVelocity()) > 0.01;
            boolean atGoal = error < 0.03;
            return atGoal && !isMoving;
        });
    }
    
    public Command moveToAlgae1() {
        return this.runOnce(() -> {
            setGoalAngle(algae1);
            isMovementEnabled = true;
        })
        .andThen(this.run(() -> {}))
        .until(() -> {
            // Only end the command once we've started moving AND reached the goal
            double error = Math.abs(m_encoder.getPosition() - m_goalAngle);
            boolean isMoving = Math.abs(m_encoder.getVelocity()) > 0.01;
            boolean atGoal = error < 0.03;
            return atGoal && !isMoving;
        });
    }
    
    public Command moveToAlgae2() {
        return this.runOnce(() -> {
            setGoalAngle(algae2);
            isMovementEnabled = true;
        })
        .andThen(this.run(() -> {}))
        .until(() -> {
            // Only end the command once we've started moving AND reached the goal
            double error = Math.abs(m_encoder.getPosition() - m_goalAngle);
            boolean isMoving = Math.abs(m_encoder.getVelocity()) > 0.01;
            boolean atGoal = error < 0.03;
            return atGoal && !isMoving;
        });
    }
    
    public Command moveToAlgae3() {
        return this.runOnce(() -> {
            setGoalAngle(algae3);
            isMovementEnabled = true;
        })
        .andThen(this.run(() -> {}))
        .until(() -> {
            // Only end the command once we've started moving AND reached the goal
            double error = Math.abs(m_encoder.getPosition() - m_goalAngle);
            boolean isMoving = Math.abs(m_encoder.getVelocity()) > 0.01;
            boolean atGoal = error < 0.03;
            return atGoal && !isMoving;
        });
    }
}