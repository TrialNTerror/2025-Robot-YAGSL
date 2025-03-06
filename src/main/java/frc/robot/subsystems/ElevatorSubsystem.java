
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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
import frc.robot.Constants.ArmConstants;
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

    private boolean locked;

public ElevatorSubsystem() {

        //Leader Motor Assigning
    elevatorLeadMotor = new SparkMax(ElevatorConstants.elevatorLeadMotorCanID, MotorType.kBrushless);    // Assigns motor 1 the CAN id (located in constants) and the motor type
    elevEncoder1 = elevatorLeadMotor.getEncoder();   
    pidController1 = elevatorLeadMotor.getClosedLoopController();                                                 // Assigns m_encoder with information from the hand motor's absolute encoder

        //Follower Motor Assigning
    elevatorFollowMotor = new SparkMax(ElevatorConstants.elevatorFollowMotor2CanID, MotorType.kBrushless);

        //Servo Motor Assigning
    leaderServo = new Servo(ElevatorConstants.servoIDLeadSide);
    followerServo = new Servo(ElevatorConstants.servoIDFollowSide);

    locked = false;

                   //ELEVATOR MOTOR 1 CONFIGUATION  (Leader)

        leadMotorConfig = new SparkMaxConfig();          //sets information for the overall motor

        leadMotorConfig
            .inverted(ElevatorConstants.leadMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.currentLimit);
        
        leadMotorConfig.closedLoop
            .outputRange(ElevatorConstants.minOutputElevator, ElevatorConstants.maxOutputElevator)
            .pid(ElevatorConstants.P, ElevatorConstants.I, ElevatorConstants.D)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            //.maxMotion
            //.maxAcceleration(ElevatorConstants.maxAcceleration)
            //.maxVelocity(ElevatorConstants.maxVelocity)
            //.allowedClosedLoopError(ElevatorConstants.allowedErr);
          
         leadMotorConfig.encoder
            .positionConversionFactor(ElevatorConstants.positionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);
         


                   //ELEVATOR MOTOR 2 CONFIGUATION  (follower)
           followMotorConfig =
           new SparkMaxConfig()         
               .follow(ElevatorConstants.elevatorLeadMotorCanID);


    elevatorLeadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  //sets the configuration to the motor

    elevatorFollowMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        //method that sets the motor to a specific spot
    private void reachHeight(double goal)
    {
        pidController1.setReference((goal),
                                    ControlType.kPosition,
                                    ClosedLoopSlot.kSlot0); 
                                    //feedforward.calculate(elevEncoder1.getVelocity()));
    }


        //COMMANDS FOR ELEVATOR

    //command for L3
    public Command level3Height()
    {
        return runOnce(() -> {
             reachHeight(ElevatorConstants.level3Height);
             System.out.println("elevator level 3");
            });
    }

    //command for L2
    public Command level2Height()
    {
        return runOnce(() -> {
            reachHeight(ElevatorConstants.level2Height);
            System.out.println("elevator level 2");
        });
    }

    //command fpr L1
    public Command level1Height()
    {
        return runOnce(() -> {
            reachHeight(ElevatorConstants.level1Height);
            System.out.println("elevator level 1");
           });
    }



    //processor position
    public Command processorHeight()
    {
        return runOnce(() -> {
            reachHeight(ElevatorConstants.processorHeight);
            System.out.println("Processor Height");
           });
    }

    //ground position
    public Command groundHeight()
    {
        return runOnce(() -> {
            reachHeight(ElevatorConstants.groundHeight);
            System.out.println("Ground Height");
           });
    }

    public Command homeHeight()
    {
        return runOnce(() -> {
            reachHeight(ElevatorConstants.homeHeight);
            System.out.println("Home Height");
           });
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Encoder pos elevator ", elevEncoder1.getPosition());
    }



    public Command lockElevator()
    {
        return run(() -> {
        leaderServo.setAngle(ElevatorConstants.servoLeaderLock);
        followerServo.setAngle(ElevatorConstants.servoFollowLock);
        locked = true;
         });
    }

    public Command unlockElevator()
    {
        return run(() -> {
        leaderServo.setAngle(ElevatorConstants.servoLeaderUnlock);
        followerServo.setAngle(ElevatorConstants.servoFollowUnlock);
        locked = false;
         });
    }
    

    
    public void simulationPeriodic()
    {
    
    }

    //Free move WITH LIMITS (WILL NOT BE USED IN COMPETITION, ONLY FOR TESTING)
   
   /* 
    public Command elevatorUp()
    {
        return run(() -> {
         elevatorLeadMotor.set(0.25);
          });
    }

    public Command elevatorDown()
    {
        return run(() -> {
         elevatorLeadMotor.set(-0.25);
          });
    }
          */
}


