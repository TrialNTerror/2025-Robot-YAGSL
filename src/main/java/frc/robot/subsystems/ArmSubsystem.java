package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
	private SparkMax armMotor1;
	private SparkClosedLoopController pidController1;
	private RelativeEncoder armEncoder1;
	private SparkBaseConfig armMotorConfig;    //should be similar to max config but allows setting leading motors.

	public ArmSubsystem(){

        	//ELEVATOR MOTOR 1 ASSIGNING
    		armMotor1 = new SparkMax(ArmConstants.armMotor1CanID, MotorType.kBrushless);    // Assigns motor 1 the CAN id (located in constants) and the motor type
    		armEncoder1 = armMotor1.getEncoder();

	                    //MOTOR 1 CONFIGUATION
           	//*******************************************//

        	armMotorConfig =
        	new SparkMaxConfig()            //sets information for the overall motor
            		.inverted(false)
            		.idleMode(IdleMode.kBrake)
            		.apply(
                		new ClosedLoopConfig()  //sets information for the controller
                		.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                		.outputRange(ArmConstants.minOutputArm, ArmConstants.maxOutputArm)
                		.pidf
                		(
                    			1.0,    //    //Gives the motor energy to drive to the set point (higher number -> higher speed)
                    			0.0,    //    //Takes the difference between the robot and set point and decides whether the robot speeds up or slows down
                    			0.0,     //    //Slows down the robot before it overshoots the target point
                                0.0

                		)
            		);

		armMotor1.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  //sets the configuration to the motor
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
        	return null;
    	}

	    //command for L3
    	public Command lv3Pos()
    	{
        	return null;
    	}

	//ground position
    	public Command groundPos()
    	{
        	return null;
    	}

	    //Free move WITH LIMITS
    	public Command armFreeMove()
    	{
        	return null;
    	}
}
