package frc.robot.subsystems;

//setting up sparkmax imports 
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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

    private SparkMax topMotor;
    private SparkMax midMotor; 
    private SparkMax lowMotor; 

    private SparkBaseConfig topMotorConfig;
    private SparkBaseConfig midMotorConfig;
    private SparkBaseConfig lowMotorConfig;


public HandSubsystem() {
    
        //MOTOR SETUP
    topMotor = new SparkMax(HandConstants.handMotor1CanID, MotorType.kBrushless);

    midMotor = new SparkMax(HandConstants.handMotor2CanID, MotorType.kBrushless); 

    lowMotor = new SparkMax(HandConstants.handMotor2CanID, MotorType.kBrushless); 



        // configuration for motor 1
    topMotorConfig =
            new SparkMaxConfig()            //sets information for the overall motor
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .apply(
                    new ClosedLoopConfig()  //sets information for the controller
                    .pid
                    (                  
                        1.0,    //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
                        0.0,    //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
                        0.0     //    //Slows down the robot before it overshoots the target point
                    )
                );


        // configuration for motor 2
    midMotorConfig =
            new SparkMaxConfig()            //sets information for the overall motor
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .apply(
                    new ClosedLoopConfig()  //sets information for the controller
                    .pid
                    (                  
                        1.0,    //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
                        0.0,    //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
                        0.0     //    //Slows down the robot before it overshoots the target point
                    )
                );


        // configuration for motor 3
    lowMotorConfig =
            new SparkMaxConfig()            //sets information for the overall motor
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .apply(
                    new ClosedLoopConfig()  //sets information for the controller
                    .pid
                    (                  
                        1.0,    //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
                        0.0,    //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
                        0.0     //    //Slows down the robot before it overshoots the target point
                    )
                );


    //set information for each motor
    topMotor.configure(topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    midMotor.configure(midMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    lowMotor.configure(lowMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        //COMMANDS

    public Command intakeCoral()   //run motor at a constant speed
    {
        return this.run(() -> {
            midMotor.set(HandConstants.intakeSpeed * HandConstants.inverted);
            lowMotor.set(HandConstants.intakeSpeed * HandConstants.inverted);
        });
    }

    public Command OutputCoral()
    {
        return this.runOnce(() -> {
            midMotor.set(-HandConstants.intakeSpeed * HandConstants.inverted);
            lowMotor.set(-HandConstants.intakeSpeed * HandConstants.inverted);
        });
    }

    public Command intakeAlgae()
    {
        return this.run(() -> {
            topMotor.set(HandConstants.intakeSpeed * HandConstants.inverted);
            midMotor.set(HandConstants.intakeSpeed * HandConstants.inverted);
        });
    }

    public Command outputAlgae()
    {
        return this.run(() -> {
            topMotor.set(-HandConstants.intakeSpeed * HandConstants.inverted);
            midMotor.set(-HandConstants.intakeSpeed * HandConstants.inverted);
        });
    }
}