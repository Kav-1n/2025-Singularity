// package frc.robot.subsystems;

// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VoltageOut;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Elevator extends SubsystemBase {
//     protected TalonFX elevatorMotor1;
//     protected TalonFX elevatorMotor2;

//     private DigitalInput bottomLimitSwitch;

//     private ProfiledPIDController pidControllerUp;
//     private ProfiledPIDController pidControllerDown;

//     private ElevatorFeedforward feedForward;

//     private double currentPIDOut;
    
//     // Control request for voltage output
//     private final VoltageOut voltageRequest = new VoltageOut(0);
    
//     public Elevator() {
//         // Create Kraken X60 motors with TalonFX controllers
//         elevatorMotor1 = new TalonFX(Constants.kElevatorMotor1ID);
//         elevatorMotor2 = new TalonFX(Constants.kElevatorMotor2ID);

//         // Configure motors
//         TalonFXConfiguration motorConfig = new TalonFXConfiguration();
//         motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kCurrentLimit;
//         motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
//         // Apply configuration to motors
//         elevatorMotor1.getConfigurator().apply(motorConfig);
//         elevatorMotor2.getConfigurator().apply(motorConfig);
        
//         // Set motor2 to follow motor1
//         elevatorMotor2.setControl(new com.ctre.phoenix6.controls.Follower(elevatorMotor1.getDeviceID(), false));

//         // Create bottom limit switch
//         bottomLimitSwitch = new DigitalInput(Constants.kBottomLimitSwitchID);

//         // Configure PID controllers
//         pidControllerUp = new ProfiledPIDController(
//             Constants.kUpControllerP, 
//             Constants.kUpControllerI,
//             Constants.kUpControllerD,
//             new TrapezoidProfile.Constraints(
//                 Constants.kMaxVelocity, 
//                 Constants.kMaxAcceleration
//             )
//         );

//         pidControllerDown = new ProfiledPIDController(
//             Constants.kDownControllerP, 
//             Constants.kDownControllerI,
//             Constants.kDownControllerD,
//             new TrapezoidProfile.Constraints(
//                 Constants.kMaxVelocity, 
//                 Constants.kMaxAcceleration
//             )
//         );

//         pidControllerUp.setTolerance(Constants.kAllowedError);
//         pidControllerDown.setTolerance(Constants.kAllowedError);

//         // Configure feedforward
//         feedForward = new ElevatorFeedforward(
//             Constants.kFeedForwardS, 
//             Constants.kFeedForwardG, 
//             Constants.kFeedForwardV
//         );
//     }

//     @Override
//     public void periodic() {
//         if (!getBottomLimitSwitch()) {
//             // Reset encoder position when at bottom limit
//             elevatorMotor1.setPosition(0);
//         }
//     }

//     /**
//      * Returns whether or not the motion is safe relative to the encoder's current position
//      * and the elevator brace position
//      * 
//      * @return Is the motion safe
//      */
//     public boolean isMotionSafe() {
//         return isMotionSafe(getEncoderPosition());
//     }

//     /**
//      * Returns whether or not the motion is safe relative to some target position and the elevator
//      * brace position
//      * 
//      * @param motionTarget The target position to determine the safety of
//      * @return Is the motion safe
//      */
//     public boolean isMotionSafe(double motionTarget) {
//         return motionTarget > Constants.kBracePosition;
//     }
    
//     /**
//      * A manual translation command that uses feed forward calculation to maintain position
//      * 
//      * @param speed The speed at which the elevator translates
//      * @return Sets motor voltage to translate the elevator and maintain position
//      */
//     public Command runManualElevator(DoubleSupplier speed) {
//         return run(() -> {
//             double desired = speed.getAsDouble();

//             if(Math.abs(MathUtil.applyDeadband(desired, .05)) > 0) {
//                 elevatorMotor1.set(speed.getAsDouble());
//             } else {
//                 elevatorMotor1.setControl(voltageRequest.withOutput(feedForward.calculate(0)));
//             }
//         });
//     }

//     /**
//      * A command that will use the feed forward to hold up the elevator.
//      * Used for feed forward tuning.
//      * 
//      * @return Sets motor voltage based on feed forward calculation.
//      */
//     public Command maintainPosition() {
//         return run(() -> {
//             elevatorMotor1.setControl(voltageRequest.withOutput(feedForward.calculate(0)));
//         });
//     }

//     public boolean eitherAtGoal() {
//         return pidControllerUp.atGoal() || pidControllerDown.atGoal();
//     }

//     /**
//      * Moves the elevator to a target destination (setpoint). 
//      * 
//      * @param setpoint Target destination of the subsystem
//      * @return Sets motor voltage to achieve the target destination
//      */
//     public Command goToSetpoint(DoubleSupplier setpoint) {
//         double clampedSetpoint = MathUtil.clamp(
//             setpoint.getAsDouble(), 
//             0, 
//             Constants.kMaxHeight
//         );

//         pidControllerDown.setGoal(clampedSetpoint);

//         return run(() -> {
//             if (!pidControllerDown.atGoal()) {
//                 currentPIDOut = pidControllerDown.calculate(
//                     getEncoderPosition(),
//                     clampedSetpoint
//                 );
//                 System.out.println("CALCULATED");
//             } else {
//                 currentPIDOut = 0;
//                 System.out.println("SET ZERO");
//             }

//             elevatorMotor1.setControl(
//                 voltageRequest.withOutput(
//                     currentPIDOut + feedForward.calculate(pidControllerDown.getSetpoint().velocity)
//                 )
//             );
//         });
//     }

//     /**
//      * Returns the current encoder position
//      * 
//      * @return Current encoder position
//      */
//     public double getEncoderPosition() {
//         return elevatorMotor1.getPosition().getValue();
//     }

//     /**
//      * Returns the value of the bottom limit switch on the elevator (false = disabled, true = enabled)
//      * 
//      * @return The value of bottomLimitSwitch
//      */
//     public boolean getBottomLimitSwitch() {
//         return bottomLimitSwitch.get();
//     }

//     /**
//      * Returns the motor's output voltage
//      * 
//      * @return Motor output voltage
//      */
//     public double getMotor1() {
//         return elevatorMotor1.getMotorVoltage().getValue();
//     }

//     /**
//      * Returns the motor's output voltage
//      * 
//      * @return Motor output voltage
//      */
//     public double getMotor2() {
//         return elevatorMotor2.getMotorVoltage().getValue();
//     }

//     public double currentPIDOut() {
//         return currentPIDOut;
//     }
// }