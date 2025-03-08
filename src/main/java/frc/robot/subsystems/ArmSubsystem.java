 package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
	private SparkFlex armMotor1;
	private SparkClosedLoopController pidController1;
	private AbsoluteEncoder armEncoder1;
	private SparkBaseConfig armMotorConfig;    //should be similar to max config but allows setting leading motors.

	// Create a new ArmFeedforward with gains kS, kG, kV, and kA

	private ArmFeedforward feedForward = new ArmFeedforward(
		ArmConstants.kStaticGain, 
		ArmConstants.kGravity,
		ArmConstants.kVelocity, 
		ArmConstants.kAccel);

	private int currentNum;

	public ArmSubsystem(){

			currentNum = 1;

        	//ELEVATOR MOTOR 1 ASSIGNING
    		armMotor1 = new SparkFlex(ArmConstants.armMotor1CanID, MotorType.kBrushless);    // Assigns motor 1 the CAN id (located in constants) and the motor type
    		armEncoder1 = armMotor1.getAbsoluteEncoder();
			pidController1 = armMotor1.getClosedLoopController();


	                    //MOTOR 1 CONFIGUATION
           	//*******************************************

        	armMotorConfig = new SparkFlexConfig();

			armMotorConfig
				.inverted(ArmConstants.motorInvert)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(ArmConstants.currentLimit);

        	armMotorConfig.closedLoop
				.outputRange(ArmConstants.minOutputArm, ArmConstants.maxOutputArm)
				.pid(ArmConstants.P, ArmConstants.I, ArmConstants.D)
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
				//.maxMotion
				//.maxAcceleration(ArmConstants.maxAcceleration)
				//.maxVelocity(ArmConstants.maxVelocity)
				//.allowedClosedLoopError(ArmConstants.allowedError);

			armMotorConfig.absoluteEncoder
				.inverted(ArmConstants.inverted)
				.positionConversionFactor(ArmConstants.positionConversionFactor)
				.velocityConversionFactor(ArmConstants.velocityConversionFactor);

			armMotorConfig.softLimit
				.forwardSoftLimit(ArmConstants.forwardLimit)
				.reverseSoftLimit(ArmConstants.reverseLimit);



		armMotor1.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  //sets the configuration to the motor
	}

		//method to reach a set point
		private void reachAngle(double goal)
    	{
        	pidController1.setReference((goal),
                	                    ControlType.kPosition,
										ClosedLoopSlot.kSlot0);
										//feedForward.calculate(armEncoder1.getPosition(), armEncoder1.getVelocity()));
		}

		private boolean endWhenArm(double cmd)
		{
			if (((cmd + ArmConstants.encoderAllowError) > cmd) && ((cmd - ArmConstants.encoderAllowError) < cmd))
			{
				return true;
			}
		    else
			{
				return false;
			}
		}


        //Switch between front and back scoring since we cant impliment vision yet

		public Command switchScore() 
		{
			return this.runOnce(() -> {
				
				if(currentNum == 1)
				{
					currentNum = -1;
					System.out.println("Back");
				}
				else
				{
					currentNum = 1;
					System.out.println("Front");
				}
			});
		}	


	    //command for Level 3
    	public Command level3Angle()
    	{
        	return runOnce(() -> {

				if (currentNum == 1)
				{
					reachAngle(ArmConstants.level3Angle);
					System.out.println("L3 Front");
					level3Angle().end(endWhenArm(ArmConstants.level3Angle));
				} 
				else
				{
					reachAngle(ArmConstants.level3BackAngle);
					System.out.println("L3 Back");
					level3Angle().end(endWhenArm(ArmConstants.level3BackAngle));
				}

			});
    	}

		
		//command for Level 2
		public Command level2Angle()
		{
			return runOnce(() -> {

				if (currentNum == 1)
				{
					reachAngle(ArmConstants.level2Angle);
					System.out.println("L2 Front");
					level2Angle().end(endWhenArm(ArmConstants.level2Angle));
				} 
				else
				{
					reachAngle(ArmConstants.level2BackAngle);
					System.out.println("L2 Back");
					level2Angle().end(endWhenArm(ArmConstants.level2BackAngle));
				}
				
			});
		}


		//command for Level 1
		public Command level1Angle()
		{
			return runOnce(() -> {

				if (currentNum == 1)
				{
					reachAngle(ArmConstants.level1Angle);
					System.out.println("L1 Front");
					level1Angle().end(endWhenArm(ArmConstants.level1Angle));
				} 
				else
				{
					reachAngle(ArmConstants.level1BackAngle);
					System.out.println("L1 Back");
					level1Angle().end(endWhenArm(ArmConstants.level1BackAngle));
				}
				
			});
		}


		//ground position
    	public Command groundAngle()
    	{
        	return runOnce(() -> 
			{
				reachAngle(ArmConstants.groundAngle);
				System.out.println("Ground");
				groundAngle().end(endWhenArm(ArmConstants.groundAngle));
			});
			
    	}

		//Home position
    	public Command homeAngle()
    	{
        	return runOnce(() -> 
			{
				reachAngle(ArmConstants.homeAngle);
				System.out.println("Home");
				homeAngle().end(endWhenArm(ArmConstants.homeAngle));
			});
    	}

		//ground position
		public Command processorAngle()
		{
			return runOnce(() -> 
			{
				reachAngle(ArmConstants.processorAngle);
				System.out.println("Processor");
				processorAngle().end(endWhenArm(ArmConstants.processorAngle));
			});
		}

    	public void periodic()
    	{
			SmartDashboard.putNumber("Encoder Pos", armEncoder1.getPosition());
    	}

	    //Free move WITH LIMITS (Probably wont use)
    	public Command armUp()
    	{
        	return runOnce(() -> armMotor1.set(0.1));
    	}

		public Command armDown()
    	{
        	return runOnce(() -> armMotor1.set(-0.1));
    	}

		public Command armStop()
    	{
        	return runOnce(() -> armMotor1.set(0));
    	}

		public Command backScore() 
		{
			return this.runOnce(() -> {
				currentNum = -1;
				System.out.println("Back");
			});
		}

		public Command frontScore() 
		{
			return this.runOnce(() -> {
				currentNum = 1;
				System.out.println("Front");
			});
		}
}