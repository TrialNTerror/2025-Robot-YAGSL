package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.Servo;


public class ElevatorSubsystem extends SubsystemBase {
    // Creates a new ExampleSubsystem. 
    //private static final MotorType kMotorType = MotorType.kBrushless;
    //private int flip = -1;
  
    //sets up a variable for motors, controllers, and encoders

    // private states it can only be used in this file
    // "SparkMax" and "AbsoluteEncoder" are the variable types, similar to int and double
    // light blue text shows the name the variable is set as

    //Leader Motor
    private SparkMax elevatorLeadMotor;
    private SparkClosedLoopController pidController1;
    private RelativeEncoder elevEncoder1;
    private SparkBaseConfig leadMotorConfig;    //should be similar to max config but allows setting leading motors.

    //Follower Motor
    private SparkMax elevatorFollowMotor;
    private SparkBaseConfig followMotorConfig;

    //Feedforward for elevator
    ElevatorFeedforward feedforward = new ElevatorFeedforward(
        ElevatorConstants.kStaticGain, 
        ElevatorConstants.kGravity, 
        ElevatorConstants.kVelocity, 
        ElevatorConstants.kAccel);

    private Servo leaderServo;
    private Servo followerServo;

public ElevatorSubsystem() {

        //ELEVATOR MOTOR 1 ASSIGNING
    elevatorLeadMotor = new SparkMax(ElevatorConstants.elevatorLeadMotorCanID, MotorType.kBrushless);    // Assigns motor 1 the CAN id (located in constants) and the motor type
    elevEncoder1 = elevatorLeadMotor.getEncoder();                                                    // Assigns m_encoder with information from the hand motor's absolute encoder

        //ELEVATOR MOTOR 2 ASSIGNING
    elevatorFollowMotor = new SparkMax(ElevatorConstants.elevatorFollowMotor2CanID, MotorType.kBrushless);

    leaderServo = new Servo(ElevatorConstants.servoIDLeadSide);
    followerServo = new Servo(ElevatorConstants.servoIDFollowSide);

                   //ELEVATOR MOTOR 1 CONFIGUATION  (Leader)

        leadMotorConfig =
        new SparkMaxConfig()            //sets information for the overall motor
            .inverted(ElevatorConstants.leadMotorInverted)
            .idleMode(IdleMode.kBrake)
            .apply(
                new ClosedLoopConfig()  //sets information for the controller
                .outputRange(ElevatorConstants.minOutputElevator, ElevatorConstants.maxOutputElevator)
                .pidf
                (                  
                    1.0,     //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
                    0.0,     //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
                    0.0,      //    //Slows down the robot before it overshoots the target point
                    0.0
                )
            );

                   //ELEVATOR MOTOR 2 CONFIGUATION  (follower)

           followMotorConfig =
           new SparkMaxConfig()            //sets information for the overall motor
               .inverted(ElevatorConstants.followMotorInverted)
               .idleMode(IdleMode.kBrake)
               .follow(ElevatorConstants.elevatorLeadMotorCanID)
               .apply(
                   new ClosedLoopConfig()  //sets information for the controller
                   .outputRange(ElevatorConstants.minOutputElevator, ElevatorConstants.maxOutputElevator)
                   .pidf
                   (                  
                       1.0,    //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
                       0.0,    //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
                       0.0,     //    //Slows down the robot before it overshoots the target point
                       0.0
                   )
               );


    elevatorLeadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  //sets the configuration to the motor

    elevatorFollowMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        //method that sets the motor to a specific spot
    private void reachHeight(double goal)
    {
        pidController1.setReference((goal),
                                    ControlType.kMAXMotionPositionControl,
                                    ClosedLoopSlot.kSlot0, 
                                    feedforward.calculate(elevEncoder1.getVelocity()));
    }


        //COMMANDS FOR ELEVATOR

    //command for L3
    public Command level3Height()
    {
        return run(() -> reachHeight(ElevatorConstants.level3Height));
    }

    //command for L2
    public Command level2Height()
    {
        return run(() -> reachHeight(ElevatorConstants.level2Height));
    }

    //command fpr L1
    public Command level1Height()
    {
        return run(() -> reachHeight(ElevatorConstants.level1Height));
    }

    //processor position
    public Command processorHeight()
    {
        return run(() -> reachHeight(ElevatorConstants.processorHeight));
    }

    //ground position
    public Command groundHeight()
    {
        return run(() -> reachHeight(ElevatorConstants.groundHeight));
    }

    public Command homeHeight()
    {
        return run(() -> reachHeight(ElevatorConstants.homeHeight));
    }

    public Command lockElevator()
    {
        return run(() -> {
        leaderServo.setAngle(ElevatorConstants.servoLeaderLock);
        followerServo.setAngle(ElevatorConstants.servoFollowLock);
         });
    }

    public Command unlockElevator()
    {
        return run(() -> {
        leaderServo.setAngle(ElevatorConstants.servoLeaderUnlock);
        followerServo.setAngle(ElevatorConstants.servoFollowUnlock);
         });
    }
    
    public void simulationPeriodic()
    {
    
    }

    //Free move WITH LIMITS (Probably won't be used for competition)
    public Command elevatorUp()
    {
        return run(() -> {
         elevatorLeadMotor.set(0.25);
         elevatorFollowMotor.set(0.25); 
          });
    }

    public Command elevatorDown()
    {
        return run(() -> {
         elevatorLeadMotor.set(-0.25);
         elevatorFollowMotor.set(-0.25); 
          });
    }
}
