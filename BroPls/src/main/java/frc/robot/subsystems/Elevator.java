package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.util.Units;

public class Elevator extends SubsystemBase {
    // Constants
    private static final double GEAR_RATIO = 15.0; // Torque increase ratio
    private static final double SPEED_REDUCTION = 7.5; // Speed reduction ratio
    
    // Define preset positions (in motor rotations)
    private static final double GROUND_INTAKE_POSITION_E = 0.0; // Placeholder
    private static final double ALGAE1_POSITION_E = 10.0; // Placeholder
    private static final double ALGAE2_POSITION_E = 20.0; // Placeholder
    private static final double BARGE_SHOOT_POSITION_E = 30.0; // Placeholder
    
    // Motor control constants
    private static final double MAX_VOLTAGE = 12.0;
    private static final double MANUAL_SPEED = 5; // Speed for manual control
    private static final double JOYSTICK_DEADBAND = 0.1;
    
    // Hardware
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    
    // Control objects
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);

    public Elevator() {
        // Initialize motors
        leaderMotor = new TalonFX(15); // Adjust IDs as needed
        followerMotor = new TalonFX(16);
        
        // Configure motors
        configureMotors();
    }
    
    private void configureMotors() {
        // Reset motors to factory defaults
        TalonFXConfiguration config = new TalonFXConfiguration();
        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);
        
        // Set neutral mode (brake when not powered)
        leaderMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Configure follower motor to follow leader
        followerMotor.setControl(new com.ctre.phoenix6.controls.Follower(leaderMotor.getDeviceID(), false));
        
        // Set position to zero
        leaderMotor.setPosition(0);
    }
    
    /**
     * Returns the current position of the elevator in motor rotations
     */
    public double getCurrentPosition() {
        return leaderMotor.getPosition().getValueAsDouble();
    }
    
    /**
     * Manually control the elevator with a joystick
     * @param speed The speed to set (-1 to 1)
     */
    public void manualControl(double speed) {
        // Apply deadband
        if (Math.abs(speed) < JOYSTICK_DEADBAND) {
            speed = 0;
        }
        
        // Apply voltage based on speed
        leaderMotor.setControl(voltageControl.withOutput(speed * MAX_VOLTAGE));
    }
    
    /**
     * Move the elevator to a specific position and hold it there
     * @param position The position to move to (in motor rotations)
     */
    public void goToPosition(double position) {
        leaderMotor.setControl(positionControl.withPosition(position));
    }
    
    /**
     * Create a command to move the elevator with a joystick
     * @param operatorXbox The operator's CommandXboxController
     * @return A command that moves the elevator based on joystick input
     */
    public Command createJoystickCommandE(CommandXboxController operatorXbox) {
        return run(() -> {
            // Get joystick value (negative is down, positive is up)
            double speed = -operatorXbox.getLeftY() * MANUAL_SPEED;
            manualControl(speed);
        });
    }
    
    /**
     * Create a command to move the elevator to the ground intake position
     * @return A command that moves the elevator to the ground intake position
     */
    public Command createGroundIntakeCommandE() {
        return run(() -> goToPosition(GROUND_INTAKE_POSITION_E));
    }
    
    /**
     * Create a command to move the elevator to the first algae position
     * @return A command that moves the elevator to the first algae position
     */
    public Command createAlgae1CommandE() {
        return run(() -> goToPosition(ALGAE1_POSITION_E));
    }
    
    /**
     * Create a command to move the elevator to the second algae position
     * @return A command that moves the elevator to the second algae position
     */
    public Command createAlgae2CommandE() {
        return run(() -> goToPosition(ALGAE2_POSITION_E));
    }
    
    /**
     * Create a command to move the elevator to the barge shooting position
     * @return A command that moves the elevator to the barge shooting position
     */
    public Command createBargeShootCommandE() {
        return run(() -> goToPosition(BARGE_SHOOT_POSITION_E));
    }
    
    /**
     * Reset the elevator position to zero
     */
    public void resetPosition() {
        leaderMotor.setPosition(0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Add any periodic checks or updates here if needed
    }
}

