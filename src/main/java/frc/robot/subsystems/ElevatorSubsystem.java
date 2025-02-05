package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    //private static final MotorType kMotorType = MotorType.kBrushless;
    //private int flip = -1;
  
    //sets up a variable for motors, controllers, and encoders

    // private states it can only be used in this file
    // "SparkMax" and "AbsoluteEncoder" are the variable types, similar to int and double
    // light blue text shows the name the variable is set as
    private SparkMax elevatorMotor1;
    private SparkClosedLoopController pidController1;
    private RelativeEncoder elevEncoder1;
    private SparkBaseConfig leadMotorConfig;    //should be similar to max config but allows setting leading motors.

    private SparkMax elevatorMotor2;
    private SparkClosedLoopController pidController2;
    private RelativeEncoder elevEncoder2;
    private SparkBaseConfig secondMotorConfig;

public ElevatorSubsystem() {
    
    ElevatorFeedforward feedforward = 
        new ElevatorFeedforward
        (
            0,
            0,
            0,
            0
        );

        //ELEVATOR MOTOR 1 ASSIGNING
    elevatorMotor1 = new SparkMax(ElevatorConstants.elevatorMotor1CanID, MotorType.kBrushless);    // Assigns motor 1 the CAN id (located in constants) and the motor type
    pidController1 = elevatorMotor1.getClosedLoopController();                                     // Assigns m_pidcontroller with information from the hand motor's closed loop controller (closed loop meaning position information is returned to us)
    elevEncoder1 = elevatorMotor1.getEncoder();                                                    // Assigns m_encoder with information from the hand motor's absolute encoder

        //ELEVATOR MOTOR 2 ASSIGNING
    elevatorMotor2 = new SparkMax(ElevatorConstants.elevatorMotor1CanID, MotorType.kBrushless);
    pidController2 = elevatorMotor2.getClosedLoopController();
    elevEncoder2 = elevatorMotor2.getEncoder();



                   //ELEVATOR MOTOR 1 CONFIGUATION
           //*******************************************//

        leadMotorConfig =
        new SparkMaxConfig()            //sets information for the overall motor
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .apply(
                new ClosedLoopConfig()  //sets information for the controller
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(ElevatorConstants.minOutputElevator, ElevatorConstants.maxOutputElevator)
                .pid
                (                  
                    1.0,    //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
                    0.0,    //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
                    0.0     //    //Slows down the robot before it overshoots the target point
                )
            );


    elevatorMotor1.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  //sets the configuration to the motor



                   //ELEVATOR MOTOR 2 CONFIGUATION
           //*******************************************//

           secondMotorConfig =
           new SparkMaxConfig()            //sets information for the overall motor
               .inverted(false)
               .idleMode(IdleMode.kBrake)
               .follow(ElevatorConstants.elevatorMotor1CanID)
               .apply(
                   new ClosedLoopConfig()  //sets information for the controller
                   .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                   .outputRange(ElevatorConstants.minOutputElevator, ElevatorConstants.maxOutputElevator)
                   .pid
                   (                  
                       1.0,    //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
                       0.0,    //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
                       0.0     //    //Slows down the robot before it overshoots the target point
                   )
               );


    elevatorMotor2.configure(secondMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }



    private void reachPos(double goal)
    {
        pidController1.setReference((goal),
                                    ControlType.kMAXMotionPositionControl);
    }

    public void simulationPeriodic()
    {
    
    }

    //command for L4
    public Command lv4Pos()
    {
        return run(() -> reachPos(1.0));
    }

    //command for L3
    public Command lv3Pos()
    {
        return run(() -> reachPos(0.75));
    }

    //command for L2
    public Command lv2Pos()
    {
        return run(() -> reachPos(0.5));
    }

    //command fpr L1
    public Command lv1Pos()
    {
        return run(() -> reachPos(0.25));
    }

    //ground position
    public Command groundPos()
    {
        return run(() -> reachPos(0));
    }

    //Free move WITH LIMITS
    public Command elevatorUp()
    {
        return run(() -> {
         elevatorMotor1.set(0.25);
         elevatorMotor2.set(0.25); 
          });
    }

    public Command elevatorDown()
    {
        return run(() -> {
         elevatorMotor1.set(-0.25);
         elevatorMotor2.set(-0.25); 
          });
    }
}