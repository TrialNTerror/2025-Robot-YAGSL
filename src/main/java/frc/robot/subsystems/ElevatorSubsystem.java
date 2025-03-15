
package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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

    private boolean endWhenElevator(double cmd)
    {
        if (((elevEncoder1.getPosition() + ElevatorConstants.encoderAllowError) > cmd) && ((elevEncoder1.getPosition() - ElevatorConstants.encoderAllowError) < cmd))
        {
            return true;
        } 
        else
        {
            return false;
        }
    }


        //COMMANDS FOR ELEVATOR

    //Level 3 position
    public class level3Height extends Command
    {
        @Override
        public void initialize(){
            reachHeight(ElevatorConstants.level3Height);
            System.out.println("elevator level 3");
        }

        @Override
        public boolean isFinished() {
            if(endWhenElevator(ElevatorConstants.level3Height) == true)
            {
                return true;
            }
            else
            {
                return false;
            }
        };
    }

    

    //Level 2 position
    public class level2Height extends Command
    {
        @Override
        public void initialize(){
            reachHeight(ElevatorConstants.level2Height);
            System.out.println("elevator level 2");
        }

        @Override
        public boolean isFinished() {
            if(endWhenElevator(ElevatorConstants.level2Height) == true)
            {
                return true;
            }
            else
            {
                return false;
            }
        };
    }

    //Level 1 position
    public class level1Height extends Command
    {
        @Override
        public void initialize(){
            reachHeight(ElevatorConstants.level1Height);
            System.out.println("elevator level 1");
        }

        @Override
        public boolean isFinished() {
            if(endWhenElevator(ElevatorConstants.level1Height) == true)
            {
                return true;
            }
            else
            {
                return false;
            }
        };
    }



    //Feeder position
    public class feederHeight extends Command
    {
        @Override
        public void initialize(){
            reachHeight(ElevatorConstants.feederHeight);
            System.out.println("Feeder Height");
        }

        @Override
        public boolean isFinished() {
            if(endWhenElevator(ElevatorConstants.feederHeight) == true)
            {
                return true;
            }
            else
            {
                return false;
            }
        };
    }

    //ground position
    public class groundHeight extends Command
    {
        @Override
        public void initialize(){
            reachHeight(ElevatorConstants.groundHeight);
            System.out.println("Ground Height (testing execute)");
        }

        @Override
        public boolean isFinished() {
            if(endWhenElevator(ElevatorConstants.groundHeight) == true)
            {
                return true;
            }
            else
            {
                return false;
            }
        };
    }


    //Home Height
    public class homeHeight extends Command
    {
        @Override
        public void initialize(){
            reachHeight(ElevatorConstants.homeHeight);
            System.out.println("Home Height (testing initialize)");
        }

        @Override
        public boolean isFinished() {
            if(endWhenElevator(ElevatorConstants.homeHeight) == true)
            {
                return true;
            }
            else
            {
                return false;
            }
        };
    }

    //Processor Height
    public class processorHeight extends Command
    {
        @Override
        public void initialize(){
            reachHeight(ElevatorConstants.processorHeight);
            System.out.println("Processor Height");
        }

        @Override
        public boolean isFinished() {
            if(endWhenElevator(ElevatorConstants.processorHeight) == true)
            {
                return true;
            }
            else
            {
                return false;
            }
        };
    }

    //initializing is for one time use per schedule, best used for setpoints
    //execute is for continuously running, best used for driving command

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
   
   
    public Command elevatorUp()
    {
        return run(() -> {
         elevatorLeadMotor.set(-0.5);
          });
    }

    public Command elevatorDown()
    {
        return run(() -> {
         elevatorLeadMotor.set(0.5);
          });
    }

    public Command elevatorStop()
    {
        return run(() -> {
         elevatorLeadMotor.set(0);
          });
    }
          
}

