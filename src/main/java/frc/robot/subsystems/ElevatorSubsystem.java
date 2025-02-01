package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private AbsoluteEncoder elevEncoder1;
    private SparkMaxConfig elevMotor1Config;

    private SparkMax elevatorMotor2;
    private SparkClosedLoopController pidController2;
    private AbsoluteEncoder elevEncoder2;
    private SparkMaxConfig elevMotor2Config;

    private SparkMax ShoulderMotor3;
    private SparkClosedLoopController pidController3;
    private AbsoluteEncoder shouEncoder3;
    private SparkMaxConfig shouMotor3Config;

public ElevatorSubsystem() {
    
        //ELEVATOR MOTOR 1 ASSIGNING
    elevatorMotor1 = new SparkMax(ElevatorConstants.elevatorMotor1CanID, MotorType.kBrushless);
    // Assigns motor 1 the CAN id (located in constants) and the motor type
    elevMotor1Config = new SparkMaxConfig();
    // Assigns motor1Config the ability to hold motor 1 properties
    pidController1 = elevatorMotor1.getClosedLoopController();
    // Assigns m_pidcontroller with information from the hand motor's closed loop controller (closed loop meaning position information is returned to us)
    elevEncoder1 = elevatorMotor1.getAbsoluteEncoder();
    // Assigns m_encoder with information from the hand motor's absolute encoder

        //ELEVATOR MOTOR 2 ASSIGNING
    elevatorMotor2 = new SparkMax(ElevatorConstants.elevatorMotor1CanID, MotorType.kBrushless);
    elevMotor2Config = new SparkMaxConfig();
    pidController2 = elevatorMotor2.getClosedLoopController();
    elevEncoder2 = elevatorMotor2.getAbsoluteEncoder();

        //SHOULDER MOTOR ASSIGNING
    elevatorMotor2 = new SparkMax(ElevatorConstants.elevatorMotor1CanID, MotorType.kBrushless);
    elevMotor2Config = new SparkMaxConfig();
    pidController2 = elevatorMotor2.getClosedLoopController();
    elevEncoder2 = elevatorMotor2.getAbsoluteEncoder();

                   //ELEVATOR MOTOR 1 CONFIGUATION
           //*******************************************//

    // configuration for motor 1
    elevMotor1Config                        //sets information for the overall motor
    .inverted(true)
    .idleMode(IdleMode.kBrake);

    elevMotor1Config.closedLoop             //sets information for the controller
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .outputRange(ElevatorConstants.minOutputElevator, ElevatorConstants.maxOutputElevator)
    .pid
    (                  
        1.0,    //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
        0.0,    //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
        0.0     //    //Slows down the robot before it overshoots the target point
    );

    elevatorMotor1.configure(elevMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                   //ELEVATOR MOTOR 2 CONFIGUATION
           //*******************************************//


    }

    //command for L4
    public Command lv4Pos()
    {
        return null;
    }

    //command for L3
    public Command lv3Pos()
    {
        return null;
    }

    //command for L2
    public Command lv2Pos()
    {
        return null;
    }

    //command fpr L1
    public Command lv1Pos()
    {
        return null;
    }

    //ground position
    public Command groundPos()
    {
        return null;
    }

    //Free move WITH LIMITS
    public Command elevatorFreeMove()
    {
        return null;
    }
}
