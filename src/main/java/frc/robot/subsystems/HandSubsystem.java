package frc.robot.subsystems;

//setting up sparkmax imports 
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;

public class HandSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    //private static final MotorType kMotorType = MotorType.kBrushless;
    //private int flip = -1;
  
    //sets up a variable for motors, controllers, and encoders

    // private states it can only be used in this file
    // "SparkMax" and "AbsoluteEncoder" are the variable types, similar to int and double
    // m_*NAME* is the name the variable is set as
    private SparkMax m_handMotor1;
    private SparkMax m_handMotor2; 
    private SparkClosedLoopController m_pidController;
    private AbsoluteEncoder m_encoder;
    private SparkMaxConfig motor1Config;

public HandSubsystem() {
    
    m_handMotor1 = new SparkMax(HandConstants.handMotor1CanID, MotorType.kBrushless); 
    // Assigns motor 1 the CAN id (located in constants) and the motor type
    motor1Config = new SparkMaxConfig();
    // Assigns motor1Config the ability to hold motor 1 properties
    m_pidController = m_handMotor1.getClosedLoopController();
    // Assigns m_pidcontroller with information from the hand motor's closed loop controller (closed loop meaning position information is returned to us)
    m_encoder = m_handMotor1.getAbsoluteEncoder();
    // Assigns m_encoder with information from the hand motor's absolute encoder


    // configuration for motor 1
    motor1Config                        //sets information for the overall motor
        .inverted(true)
        .idleMode(IdleMode.kBrake);

    motor1Config.encoder                //sets information for the encoder only
        .positionConversionFactor(1000)
        .velocityConversionFactor(1000);

    motor1Config.closedLoop             //sets information for the controller
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid
        (
            1.0,    //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
            0.0,    //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
            0.0     //    //Slows down the robot before it overshoots the target point
        );



    m_handMotor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //connects the hand motor to the configured properties 


    }
}