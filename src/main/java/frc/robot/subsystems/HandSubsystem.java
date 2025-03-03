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
    // Creates a new ExampleSubsystem. 
    //private static final MotorType kMotorType = MotorType.kBrushless;
    //private int flip = -1;
  
    //sets up a variable for motors, controllers, and encoders

    // private states it can only be used in this file
    // "SparkMax" and "AbsoluteEncoder" are the variable types, similar to int and double
    // m_*NAME* is the name the variable is set as

    private SparkMax topMotor;
    private SparkMax bottomMotor; 
    private SparkMax holdMotor; 

    private SparkBaseConfig topMotorConfig;
    private SparkBaseConfig bottomMotorConfig;
    private SparkBaseConfig holdMotorConfig;


public HandSubsystem() {
    
        //MOTOR SETUP
    topMotor = new SparkMax(HandConstants.topHandCanID, MotorType.kBrushless);

    bottomMotor = new SparkMax(HandConstants.bottomHandCanID, MotorType.kBrushless); 

    holdMotor = new SparkMax(HandConstants.holdHandCanID, MotorType.kBrushless); 



        // configuration for motor 1
    topMotorConfig =
            new SparkMaxConfig()            //sets information for the overall motor
                .inverted(HandConstants.invertAllMotors)
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
    bottomMotorConfig =
            new SparkMaxConfig()            //sets information for the overall motor
                .inverted(HandConstants.invertAllMotors)
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
    holdMotorConfig =
            new SparkMaxConfig()            //sets information for the overall motor
                .inverted(HandConstants.invertAllMotors)
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
    bottomMotor.configure(bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    holdMotor.configure(holdMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

        //COMMANDS

    public Command intakeCoral()   //run motor at a constant speed
    {
        return this.runOnce(() -> {
            bottomMotor.set(-HandConstants.intakeSpeed);
            holdMotor.set(HandConstants.intakeSpeed);
        });
    }

    public Command OutputCoral()
    {
        return this.runOnce(() -> {
            bottomMotor.set(HandConstants.intakeSpeed);
            holdMotor.set(-HandConstants.intakeSpeed);
        });
    }

    public Command intakeAlgae()
    {
        return this.runOnce(() -> {
            topMotor.set(HandConstants.intakeSpeed);
            bottomMotor.set(HandConstants.intakeSpeed);
        });
    }

    public Command outputAlgae()
    {
        return this.runOnce(() -> {
            topMotor.set(-HandConstants.intakeSpeed);
            bottomMotor.set(-HandConstants.intakeSpeed);
        });
    }

    public Command motorsOff()
    {
        return this.runOnce(() -> {
            topMotor.set(0);
            bottomMotor.set(0);
            holdMotor.set(0);
        });
    }


}
    
