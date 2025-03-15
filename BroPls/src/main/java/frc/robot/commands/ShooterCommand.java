package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand {
    private final ShooterSubsystem shooter;
    private final CommandXboxController operatorXbox;
    
    public ShooterCommand(ShooterSubsystem shooter, CommandXboxController operatorXbox) {
        this.shooter = shooter;
        this.operatorXbox = operatorXbox;
    }
    
    /**
     * Configure shooter commands and bind them to controller inputs
     */
    public void configureButtonBindings() {
        // Set the default command for the shooter to be controlled by triggers
        shooter.setDefaultCommand(shooter.createTriggerControlCommand(operatorXbox));
    }
    
    /**
     * Create a command that activates intake mode
     * @return A command that activates the intake
     */
    public Command createIntakeCommand() {
        return shooter.run(shooter::activateIntake);
    }
    
    /**
     * Create a command that activates shooting mode
     * @return A command that activates shooting
     */
    public Command createShootCommand() {
        return shooter.run(shooter::activateShooting);
    }
    
    /**
     * Create a command that stops the shooter
     * @return A command that stops all shooter motors
     */
    public Command createStopCommand() {
        return shooter.runOnce(shooter::stopMotors);
    }
    
    /**
     * Create a command to shoot for a specified duration in seconds
     * @param durationSeconds How long to shoot for
     * @return A timed command that shoots then stops
     */
    public Command createTimedShootCommand(double durationSeconds) {
        return shooter.run(shooter::activateShooting)
            .withTimeout(durationSeconds)
            .andThen(shooter.runOnce(shooter::stopMotors));
    }
    
    /**
     * Create a command to intake for a specified duration in seconds
     * @param durationSeconds How long to intake for
     * @return A timed command that intakes then stops
     */
    public Command createTimedIntakeCommand(double durationSeconds) {
        return shooter.run(shooter::activateIntake)
            .withTimeout(durationSeconds)
            .andThen(shooter.runOnce(shooter::stopMotors));
    }
}

