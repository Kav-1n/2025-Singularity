package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    // Define motor CAN IDs
    private static final int LEFT_MOTOR_ID = 17;
    private static final int RIGHT_MOTOR_ID = 18;
    
    // Define motor speed values
    private static final double INTAKE_SPEED = 0.3;
    private static final double SHOOTING_SPEED = 1;

    // Maximum voltage available
    private static final double MAX_VOLTAGE = 12.0;

    // Create motor objects
    private final TalonFX leftMotor = new TalonFX(LEFT_MOTOR_ID);
    private final TalonFX rightMotor = new TalonFX(RIGHT_MOTOR_ID);
    
    // Create duty cycle control object
    private final DutyCycleOut speedControl = new DutyCycleOut(0);
    
    /**
     * Constructor
     */
    public ShooterSubsystem() {
        // Configure motors for maximum power output
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limits - increase to maximum safe values
        config.CurrentLimits.SupplyCurrentLimit = 80; // Significantly higher than default
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 150; // Very high to prevent stalling
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Voltage configuration for maximum power
        config.Voltage.PeakForwardVoltage = MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_VOLTAGE;
        config.Voltage.SupplyVoltageTimeConstant = 0.01; // Fast voltage compensation
        
        // Zero ramp-up time for instant maximum power
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
        
        // Set to Coast mode when stopping to prevent sudden braking
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Apply configuration to both motors
        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);
    }
    
    /**
     * Run the intake (motors run inward)
     */
    public void intake() {
        leftMotor.setControl(speedControl.withOutput(INTAKE_SPEED));
        rightMotor.setControl(speedControl.withOutput(-INTAKE_SPEED));
    }
    
    /**
     * Run the shooter (motors run outward) with maximum available power
     */
    public void shoot() {
        // Use the same speed but with optimized power delivery
        leftMotor.setControl(speedControl.withOutput(-SHOOTING_SPEED));
        rightMotor.setControl(speedControl.withOutput(SHOOTING_SPEED));
    }
    
    /**
     * Stop all shooter motors
     */
    public void stop() {
        leftMotor.setControl(speedControl.withOutput(0));
        rightMotor.setControl(speedControl.withOutput(0));
    }
}