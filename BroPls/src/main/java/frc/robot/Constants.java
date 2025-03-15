// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(10);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static class AutoNames{
    public static final String LEAVEAUTO = "LeaveAuto";
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static final int kElevatorMotor1ID = 15;
  public static final int kElevatorMotor2ID = 16;

  public static final int kBottomLimitSwitchID = 0;

  // 60/11 gearing multiplied by circumference of sprocket multiplied by 2 for carriage position
  public static final double kEncoderPositionConversionFactor = 11.0/60.0 * (22.0*0.25) * 2.0;
  public static final double kEncoderVelocityConversionFactor = kEncoderPositionConversionFactor / 60;

  public static final int kCurrentLimit = 40;

  public static final double kUpControllerP = 5.6;//7; //
  public static final double kUpControllerI = 0;
  public static final double kUpControllerD = 0.28;//0.28

  public static final double kDownControllerP = 5.6;//7; //
  public static final double kDownControllerI = 0;
  public static final double kDownControllerD = 0.57;//0.175;//0.1;//0.35
   
  public static final double kAllowedError = 1;

  public static final double kFeedForwardS = (0.95 - 0.2)/2*0.8;   /* kG too high - kG too low / 2  0.95, 0.2 */
  public static final double kFeedForwardG = (0.95 + 0.2)/2;  /* kG too high + kG too low / 2 */    // calculated value 0.6
  public static final double kFeedForwardV = 0.12;   // calculated value 0.12

  public static final double kMaxVelocity = 150.0;    // 120 inches per second (COOKING) calculated max is 184 in/s
  public static final double kMaxAcceleration = 240;    // 400 inches per second^2 (also COOKING) calculated max is 600 in/s^2

  public static final double kCoralIntakePosition = 0;
  public static final double kL1Position = 0;
  public static final double kL2Position = 14.5;
  public static final double kL3Position = 29.0;
  public static final double kL4Position = 53.0;
  public static final double kL4TransitionPosition = 40.0;
  public static final double kL2AlgaePosition = 22.0;
  public static final double kL3AlgaePosition = 39.0;
  public static final double kProcessorPosition = 4.0;
  /**The position of the top of the elevator brace */
  public static final double kBracePosition = 5.5;
  public static final double kMaxHeight = 47.5; //actual is 53

  // 1, 7, 10 are the defaults for these, change as necessary
  public static final double kSysIDRampRate = .25;
  public static final double kSysIDStepVolts = 3;
  public static final double kSysIDTimeout = 10;

  public static final IdleMode kIdleMode = IdleMode.kBrake;

  // YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW THIS LINE UNLESS YOU'RE ADDING A NEW CONFIGURATION ITEM
  
  public static final SysIdRoutine.Config kSysIDConfig = new Config(
      Volts.of(kSysIDRampRate).per(Second), 
      Volts.of(kSysIDStepVolts), 
      Seconds.of(kSysIDTimeout)
  );

  public static final SparkMaxConfig motorConfig = new SparkMaxConfig();
  
  static {
      motorConfig
          .smartCurrentLimit(kCurrentLimit)
          .idleMode(kIdleMode)
          .inverted(true);
      motorConfig.encoder
          .positionConversionFactor(kEncoderPositionConversionFactor)
          .velocityConversionFactor(kEncoderVelocityConversionFactor);


          }

          public static class MotorControllers {
            public static final int SMART_CURRENT_LIMIT = 40;
           //Motor ID Numbers
            //Elevator 
            public static final int ID_ELEVATOR_LEFT_TALON = 15;
            public static final int ID_ELEVATOR_RIGHT_TALON = 16;
          }        


        


}
