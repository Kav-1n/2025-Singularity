package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    // Motor speeds
    private static final double INTAKE_SPEED = 0.7; // 70% speed for intake
    private static final double SHOOTING_SPEED = 0.9; // 90% speed for shooting
    
    // Motor control constants
    private static final double MAX_VOLTAGE = 12.0;
    private static final double TRIGGER_THRESHOLD = 0.1; // Deadband for triggers
    
    // Hardware
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    
    // Control objects
    private final VoltageOut voltageControl = new VoltageOut(0);
    
    // Operational state
    private boolean isIntaking = false;
    private boolean isShooting = false;

    public ShooterSubsystem() {
        // Initialize motors
        leftMotor = new TalonFX(17);
        rightMotor = new TalonFX(18);
        
        // Configure motors
        configureMotors();
    }
    
    private void configureMotors() {
        // Create and apply configuration for both motors
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Set current limits to protect motors
        config.CurrentLimits.SupplyCurrentLimit = 40; // 40 amps
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Apply configurations
        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);
        
        // Set neutral mode (coast when not powered for less wear on gears)
        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    
    /**
     * Activate intake mode (left motor clockwise, right motor counterclockwise)
     */
    public void activateIntake() {
        // For TalonFX, positive is counterclockwise when looking at the shaft
        // So we need to invert the directions based on motor orientation
        leftMotor.setControl(voltageControl.withOutput(-INTAKE_SPEED * MAX_VOLTAGE));
        rightMotor.setControl(voltageControl.withOutput(INTAKE_SPEED * MAX_VOLTAGE));
        isIntaking = true;
        isShooting = false;
    }
    
    /**
     * Activate shooting mode (left motor counterclockwise, right motor clockwise)
     */
    public void activateShooting() {
        leftMotor.setControl(voltageControl.withOutput(SHOOTING_SPEED * MAX_VOLTAGE));
        rightMotor.setControl(voltageControl.withOutput(-SHOOTING_SPEED * MAX_VOLTAGE));
        isIntaking = false;
        isShooting = true;
    }
    
    /**
     * Stop all motors
     */
    public void stopMotors() {
        leftMotor.setControl(voltageControl.withOutput(0));
        rightMotor.setControl(voltageControl.withOutput(0));
        isIntaking = false;
        isShooting = false;
    }
    
    /**
     * Create a command to control the shooter based on controller triggers
     * @param operatorXbox The operator's CommandXboxController
     * @return A command that controls the shooter based on trigger inputs
     */
    public Command createTriggerControlCommand(CommandXboxController operatorXbox) {
        return run(() -> {
            double leftTrigger = operatorXbox.getLeftTriggerAxis();
            double rightTrigger = operatorXbox.getRightTriggerAxis();
            
            // Check right trigger first (shooting takes priority)
            if (rightTrigger > TRIGGER_THRESHOLD) {
                activateShooting();
            } 
            // Then check left trigger for intake
            else if (leftTrigger > TRIGGER_THRESHOLD) {
                activateIntake();
            } 
            // If neither trigger is pressed, stop motors
            else {
                stopMotors();
            }
        });
    }
    
    @Override
    public void periodic() {
        // Update dashboard with shooter status
        SmartDashboard.putBoolean("Shooter Intaking", isIntaking);
        SmartDashboard.putBoolean("Shooter Shooting", isShooting);
        SmartDashboard.putNumber("Left Motor Output", leftMotor.get());
        SmartDashboard.putNumber("Right Motor Output", rightMotor.get());
    }
}

