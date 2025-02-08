package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
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

	private int currentNum;

	public ArmSubsystem(){

			currentNum = 1;

        	//ELEVATOR MOTOR 1 ASSIGNING
    		armMotor1 = new SparkMax(ArmConstants.armMotor1CanID, MotorType.kBrushless);    // Assigns motor 1 the CAN id (located in constants) and the motor type
    		armEncoder1 = armMotor1.getEncoder();


	                    //MOTOR 1 CONFIGUATION
           	//*******************************************//

        	armMotorConfig =
        	new SparkMaxConfig()            //sets information for the overall motor
            		.inverted(ArmConstants.motorInvert)
            		.idleMode(IdleMode.kBrake)
            		.apply(
                		new ClosedLoopConfig()  //sets information for the controller
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

		//method to reach a set point
		private void reachAngle(double goal)
    	{
        	pidController1.setReference((goal),
                	                    ControlType.kMAXMotionPositionControl);
		}


        //Switch between front and back scoring since we cant impliment vision yet
		public Command backScore() 
		{
			return this.run(() -> {
				currentNum = -1;
			});
		}
	
		public Command frontScore() 
		{
			return this.run(() -> {
				currentNum = 1;
			});
		}


	    //command for Level 3
    	public Command level3Angle()
    	{
        	return run(() -> {

				if (currentNum == 1)
				{
					reachAngle(ArmConstants.level3Angle);
				} 
				else
				{
					reachAngle(ArmConstants.level3BackAngle);
				}

			});
    	}

		
		//command for Level 2
		public Command level2Angle()
		{
			return run(() -> {

				if (currentNum == 1)
				{
					reachAngle(ArmConstants.level2Angle);
				} 
				else
				{
					reachAngle(ArmConstants.level2BackAngle);
				}
				
			});
		}


		//command for Level 1
		public Command level1Angle()
		{
			return run(() -> {

				if (currentNum == 1)
				{
					reachAngle(ArmConstants.level1Angle);
				} 
				else
				{
					reachAngle(ArmConstants.level1BackAngle);
				}
				
			});
		}


		//ground position
    	public Command groundAngle()
    	{
        	return run(() -> reachAngle(ArmConstants.groundAngle));
    	}

		//Home position
		//ground position
    	public Command homeAngle()
    	{
        	return run(() -> reachAngle(ArmConstants.homeAngle));
    	}

    	public void simulationPeriodic()
    	{
    	
    	}

	    //Free move WITH LIMITS (Probably wont use)
    	public Command armFreeMove()
    	{
        	return null;
    	}
}
