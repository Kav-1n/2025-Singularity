package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        m_goalAngle = m_encoder.getPosition(); // Set initial goal to current position
        
        TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(1, 10);
        m_controller = new ProfiledPIDController(1.5, 0.0, 0.1, m_constraints, deltaTime);
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
            pivotMotor.set(0);
            return;
        }
        
        m_controller.setGoal(m_goalAngle);
        
        // Compute motor output using PID
        double speed = m_controller.calculate(m_encoder.getPosition());
    
        // Limit speed to prevent high-torque issues
        speed = Math.max(-0.2, Math.min(0.2, speed));
    
        // Apply speed to motor
        pivotMotor.set(speed);
    
        // Stop motor if it's already at the target
        if (isPivotAtGoal()) {
            pivotMotor.stopMotor();
        }
    
        SmartDashboard.putNumber("Pivot Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Pivot Speed", speed);
    }
    

    

    public enum Positions {
        GROUND_INTAKE,
        ALGAE1,
        ALGAE2,
        ALGAE3
    }

    public void goToAngle(Positions position) {
        if (!isPivotAtGoal()) { // Prevent changing targets rapidly
            switch (position) {
                case GROUND_INTAKE:
                    setGoalAngle(groundIntake);
                    break;
                case ALGAE1:
                    setGoalAngle(algae1);
                    break;
                case ALGAE2:
                    setGoalAngle(algae2);
                    break;
                case ALGAE3:
                    setGoalAngle(algae3);
                    break;
            }
        }
    }

  public boolean isPivotAtGoal() {
    return Math.abs(m_encoder.getPosition() - m_goalAngle) < 0.03;
  }

  public Command moveToGroundIntake() {
    return this.run(() -> goToAngle(Positions.GROUND_INTAKE));
  }

  public Command moveToAlgae1() {
    return this.run(() -> goToAngle(Positions.ALGAE1));
  }

  public Command moveToAlgae2() {
    return this.run(() -> goToAngle(Positions.ALGAE2));
  }

  public Command moveToAlgae3() {
    return this.run(() -> goToAngle(Positions.ALGAE3));
  }
}
