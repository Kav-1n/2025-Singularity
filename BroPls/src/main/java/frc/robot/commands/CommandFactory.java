package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotSubsystem;


public class CommandFactory {
    private final Elevator elevator;
    private final PivotSubsystem pivot;
    public CommandXboxController operatorXbox = new CommandXboxController(1);
        
        public CommandFactory(Elevator elevator, PivotSubsystem pivot, CommandXboxController operatorXbox) {
            this.elevator = elevator;
            this.pivot = pivot;
            this.operatorXbox = operatorXbox;
    }
    
    /**
     * Configure all commands and bind them to controller inputs
     */
    public void configureButtonBindings() {
        // Set default commands for manual control
        elevator.setDefaultCommand(elevator.createJoystickCommandE(operatorXbox));
        pivot.setDefaultCommand(pivot.createJoystickCommand(operatorXbox));
        
        // Create combined preset commands
        Command groundIntakeCommand = new ParallelCommandGroup(
            elevator.createGroundIntakeCommandE(),
            pivot.createGroundIntakeCommand()
        );
        
        Command algae1Command = new ParallelCommandGroup(
            elevator.createAlgae1CommandE(),
            pivot.createAlgae1Command()
        );
        
        Command algae2Command = new ParallelCommandGroup(
            elevator.createAlgae2CommandE(),
            pivot.createAlgae2Command()
        );
        
        Command bargeShootCommand = new ParallelCommandGroup(
            elevator.createBargeShootCommandE(),
            pivot.createBargeShootCommand()
        );
        
        // Bind preset positions to buttons (A, X, B, Y)
        operatorXbox.a()
            .onTrue(groundIntakeCommand);
            
        operatorXbox.x()
            .onTrue(algae1Command);
            
        operatorXbox.b()
            .onTrue(algae2Command);
            
        operatorXbox.y()
            .onTrue(bargeShootCommand);
            
        // Reset encoder when back button is pressed
        operatorXbox.back()
            .onTrue(pivot.createResetEncoderCommand());
    }
}



