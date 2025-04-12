package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Elevator extends SubsystemBase {
    // Define preset positions (in motor rotations)
    private static final double GROUND_POSITION = 0.0;
    private static final double ALGAE1_POSITION = 19.0;
    private static final double ALGAE2_POSITION = 27.0;
    private static final double BARGE_POSITION = 33.0;
    // New preset positions
    private static final double TROUGH_POSITION = 24.0;
    private static final double L2_POSITION = 24.0;
    private static final double L3_POSITION = 33.0;
    private static final double L4_POSITION = 33.0;
    
    // Constants for movement speeds (significantly increased)
    private static final double SLOW_UP_SPEED = 0.30;    // Increased upward speed
    private static final double SLOW_DOWN_SPEED = -0.15; // Increased downward speed (still conservative for safety)
    
    // Motor control constants
    private static final double JOYSTICK_DEADBAND = 0.1;
    
    // Position hold PID values (increased for better response with higher speeds)
    private static final double POSITION_HOLD_KP = 10;
    private static final double POSITION_HOLD_KI = 1.5;
    private static final double POSITION_HOLD_KD = 2.0;
    private static final double POSITION_HOLD_KS = 0.7;   // Static friction compensation
    private static final double POSITION_HOLD_KV = 0.2;  // Velocity feedforward
    
    // Maximum voltage output
    private static final double MAX_VOLTAGE = 15.0;
    
    // Speed reduction factor for manual control
    private static final double MANUAL_SPEED_FACTOR = 0.8; // Increased to 80% of full speed
    
    // Hardware
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    
    // Control objects
    private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    // Current target position for position holding
    private double currentTargetPosition = 0.0;
    private boolean isHoldingPosition = false;
    private boolean isManualControlActive = false;
    
    // Voltage configurations
    private final VoltageConfigs normalVoltageConfig = new VoltageConfigs();
    private final VoltageConfigs lowVoltageConfig = new VoltageConfigs();

    public Elevator() {
        // Initialize motors
        leaderMotor = new TalonFX(15);
        followerMotor = new TalonFX(16);
        
        // Base motor configuration
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Configure slot 0 for position holding
        // Configure slot 0 for position holding with more advanced gains
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = POSITION_HOLD_KP;
        slot0.kI = POSITION_HOLD_KI;
        slot0.kD = POSITION_HOLD_KD;
        slot0.kS = POSITION_HOLD_KS;  // Static friction compensation
        slot0.kV = POSITION_HOLD_KV;  // Velocity feedforward
        config.Slot0 = slot0;
        
        // Set smoothing ramp rates
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5; // Reduced from 800ms to 500ms for faster response
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.3;   // Reduced from 400ms to 300ms for faster response
        
        // Set current limits for safety
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Set output voltage to absolute maximum (for manual control)
        config.Voltage.PeakForwardVoltage = MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -MAX_VOLTAGE;
        
        // Apply configuration
        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);
        
        // Set strongest possible brake mode
        leaderMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Configure follower
        followerMotor.setControl(new com.ctre.phoenix6.controls.Follower(leaderMotor.getDeviceID(), false));
        
        // Set position to zero
        leaderMotor.setPosition(0);
        currentTargetPosition = 0;
        
        // Configure voltage configs for switching between normal and low power mode
        normalVoltageConfig.PeakForwardVoltage = MAX_VOLTAGE;
        normalVoltageConfig.PeakReverseVoltage = -MAX_VOLTAGE;
        
        // Increased voltage for faster movements while maintaining safety
        lowVoltageConfig.PeakForwardVoltage = 8.0;  // Increased from 5.0V to 8.0V
        lowVoltageConfig.PeakReverseVoltage = -8.0; // Increased from -5.0V to -8.0V
    }
    
    /**
     * Set the elevator to a specific position
     * @param position The target position to move to
     */
    public void setPosition(double position) {
        // Reset manual control flags
        isManualControlActive = false;
        isHoldingPosition = true;
        
        // Apply low voltage config for safety
        leaderMotor.getConfigurator().apply(lowVoltageConfig);
        
        // Set the target position
        currentTargetPosition = position;
        
        // Use position control to move to the specified position with increased feedforward
        leaderMotor.setControl(positionControl.withPosition(position)
                                            .withFeedForward(0.08)); // Increased from 0.05 to 0.08
        
        System.out.println("Elevator moving to position: " + position);
    }
    
    /**
     * Manually control the elevator with a joystick value
     */
    public void setManualSpeed(double speed) {
        // Apply deadband
        if (Math.abs(speed) < JOYSTICK_DEADBAND) {
            // If joystick is at neutral, hold current position
            if (isManualControlActive) {
                // We just released the joystick, capture current position
                currentTargetPosition = getCurrentPosition();
                isManualControlActive = false;
                isHoldingPosition = true;
            }
            
            // Apply position control to hold at current position
            if (isHoldingPosition) {
                leaderMotor.setControl(positionControl.withPosition(currentTargetPosition)
                                                    .withFeedForward(0.08)); // Increased from 0.05 to 0.08
            }
        } else {
            // Joystick is active - use duty cycle control
            isManualControlActive = true;
            isHoldingPosition = false;
            
            // Make sure we're in normal voltage mode
            leaderMotor.getConfigurator().apply(normalVoltageConfig);
            
            // Apply speed reduction factor for better control while maintaining torque
            double scaledSpeed = speed * MANUAL_SPEED_FACTOR;
            
            // Add gravity compensation based on direction
            double outputPower = scaledSpeed;
            
            if (scaledSpeed > 0) {
                // Moving up - add extra power to overcome gravity
                outputPower = scaledSpeed + 0.15; // Increased from 0.10 to 0.15 for more compensation
                
                // Limit to maximum of the speed factor to maintain control
                outputPower = Math.min(outputPower, MANUAL_SPEED_FACTOR);
            } else {
                // Going down - be careful not to drop too fast
                outputPower = Math.max(scaledSpeed, -MANUAL_SPEED_FACTOR);
            }
            
            // Apply direct duty cycle output
            leaderMotor.setControl(dutyCycleControl.withOutput(outputPower));
            
            // Update current position for smooth transition to hold
            currentTargetPosition = getCurrentPosition();
        }
    }
    
    /**
     * Get current motor position
     */
    public double getCurrentPosition() {
        return leaderMotor.getPosition().getValueAsDouble();
    }
    
    /**
     * Move the elevator upward at a slow, safe speed
     */
    public void moveSlowUp() {
        isManualControlActive = false;
        isHoldingPosition = false;
        
        // Apply low voltage config for safety
        leaderMotor.getConfigurator().apply(lowVoltageConfig);
        
        // Set a slow, constant upward speed
        leaderMotor.setControl(dutyCycleControl.withOutput(SLOW_UP_SPEED));
        
        System.out.println("Elevator moving up at speed: " + SLOW_UP_SPEED);
    }
    
    /**
     * Move the elevator downward at a slow, safe speed
     */
    public void moveSlowDown() {
        isManualControlActive = false;
        isHoldingPosition = false;
        
        // Apply low voltage config for safety
        leaderMotor.getConfigurator().apply(lowVoltageConfig);
        
        // Set a slow, constant downward speed
        leaderMotor.setControl(dutyCycleControl.withOutput(SLOW_DOWN_SPEED));
        
        System.out.println("Elevator moving down at speed: " + SLOW_DOWN_SPEED);
    }
    
    /**
     * Stop elevator movement and hold position
     */
    public void stopAndHold() {
        // Capture current position
        currentTargetPosition = getCurrentPosition();
        isManualControlActive = false;
        isHoldingPosition = true;
        
        // Apply position control to hold at current position
        leaderMotor.setControl(positionControl.withPosition(currentTargetPosition)
                                            .withFeedForward(0.08)); // Increased from 0.05 to 0.08
        
        System.out.println("Elevator stopped and holding at position: " + currentTargetPosition);
    }
    
    /**
     * Reset the elevator position to zero
     */
    public void resetPosition() {
        System.out.println("Resetting elevator position to zero");
        leaderMotor.setPosition(0);
        currentTargetPosition = 0;
        isHoldingPosition = true;
        isManualControlActive = false;
    }

    /**
     * Creates a command to move to a preset height and stay there even after button release
     * @param targetHeight The preset height to stop at
     * @return Command for moving to preset height and staying there
     */
    public Command createPresetHeightCommand(double targetHeight) {
        return Commands.sequence(
            // First apply low voltage config and ensure we can move
            Commands.runOnce(() -> {
                System.out.println("Starting move to preset height: " + targetHeight);
                isHoldingPosition = false;
                isManualControlActive = false;
                leaderMotor.getConfigurator().apply(lowVoltageConfig);
                
                // Apply an initial pulse to overcome static friction
                double direction = Math.signum(targetHeight - getCurrentPosition());
                if (direction != 0) {
                    leaderMotor.setControl(dutyCycleControl.withOutput(direction * 0.3));
                }
            }),
            
            // Brief delay for config and pulse to apply
            Commands.waitSeconds(0.1),
            
            // Move until target height
            Commands.run(() -> {
                double currentPosition = getCurrentPosition();
                
                // Log position progress for debugging (approximately every 0.5 seconds)
                if (Math.random() < 0.02) { 
                    System.out.println("Elevator moving... Current: " + currentPosition + ", Target: " + targetHeight);
                }
                
                if (currentPosition < targetHeight) {
                    moveSlowUp();
                } else if (currentPosition > targetHeight + 0.5) {
                    moveSlowDown();
                } else {
                    // Close enough, switch to position control
                    setPosition(targetHeight);
                }
            }, this)
            .until(() -> Math.abs(getCurrentPosition() - targetHeight) < 0.5),
            
            // Stop and hold when target reached
            Commands.runOnce(() -> {
                System.out.println("Elevator reached target, holding at: " + targetHeight);
                // Explicitly set the position and hold it
                setPosition(targetHeight);
                isHoldingPosition = true;
                isManualControlActive = false;
            }, this),
            
            // Wait while position is held - more explicit holding
            Commands.run(() -> {
                // Continuously set the position to ensure it stays in place
                leaderMotor.setControl(positionControl
                    .withPosition(targetHeight)
                    .withFeedForward(0.08) // Increased from 0.05 to 0.08
                );
            }, this)
        ).finallyDo((interrupted) -> {
            // When button is released or command interrupted, maintain current position
            System.out.println("Button released, maintaining position at: " + getCurrentPosition());
            
            // Get current position
            double currentPos = getCurrentPosition();
            
            // Create and schedule a command to hold at current position
            Commands.runOnce(() -> {
                setPosition(currentPos);
                isHoldingPosition = true;
                isManualControlActive = false;
            }, this).schedule();
        });
    }
    
    /**
     * Creates command to zero elevator encoder
     */
    public Command createZeroEncoderCommand() {
        return Commands.runOnce(() -> {
            System.out.println("Zeroing elevator encoder");
            resetPosition();
        }, this);
    }
    
    /**
     * Creates a command that moves to zero position
     */
    public Command createMoveToZeroCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("Moving to zero position");
                leaderMotor.getConfigurator().apply(lowVoltageConfig);
            }),
            Commands.waitSeconds(0.05),
            Commands.run(
                this::moveSlowDown,
                this
            ).until(() -> Math.abs(getCurrentPosition()) < 0.5),
            Commands.runOnce(this::stopAndHold, this)
        );
    }

    /**
     * Creates command for Algae1 position
     */
    public Command createAlgae1Command() {
        return createPresetHeightCommand(ALGAE1_POSITION);
    }
    
    /**
     * Creates command for Algae2 position
     */
    public Command createAlgae2Command() {
        return createPresetHeightCommand(ALGAE2_POSITION);
    }
    
    /**
     * Creates command for Barge position
     */
    public Command createBargeCommand() {
        return createPresetHeightCommand(BARGE_POSITION);
    }
    
    /**
     * Creates command for Trough position
     */
    public Command createTroughCommand() {
        return createPresetHeightCommand(TROUGH_POSITION);
    }
    
    /**
     * Creates command for L2 position
     */
    public Command createL2Command() {
        return createPresetHeightCommand(L2_POSITION);
    }
    
    /**
     * Creates command for L3 position
     */
    public Command createL3Command() {
        return createPresetHeightCommand(L3_POSITION);
    }
    
    /**
     * Creates command for L4 position
     */
    public Command createL4Command() {
        return createPresetHeightCommand(L4_POSITION);
    }
    
    @Override
    public void periodic() {
        // If we're supposed to be holding position but not in manual control,
        // ensure we're still holding that position
        if (isHoldingPosition && !isManualControlActive) {
            leaderMotor.setControl(positionControl.withPosition(currentTargetPosition)
                                                .withFeedForward(0.08)); // Increased from 0.05 to 0.08
        }
    }
}