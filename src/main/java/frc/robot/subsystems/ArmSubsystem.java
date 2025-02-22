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
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
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
				.inverted(ArmConstants.inverted)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(ArmConstants.stallLimit);

        	armMotorConfig.closedLoop
				.outputRange(ArmConstants.minOutputArm, ArmConstants.maxOutputArm)
				.pid(ArmConstants.P, ArmConstants.I, ArmConstants.D)
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.maxMotion
				.maxAcceleration(ArmConstants.maxAcceleration)
				.maxVelocity(ArmConstants.maxVelocity)
				.allowedClosedLoopError(ArmConstants.allowedErr);

			armMotorConfig.encoder
				.inverted(ArmConstants.inverted)
				.positionConversionFactor(ArmConstants.positionConversionFactor)
				.velocityConversionFactor(ArmConstants.velocityConversionFactor);


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
        	return runOnce(() -> reachAngle(ArmConstants.groundAngle));
    	}

		//Home position
    	public Command homeAngle()
    	{
        	return runOnce(() -> reachAngle(ArmConstants.homeAngle));
    	}

		//ground position
		public Command processorAngle()
		{
			return run(() -> reachAngle(ArmConstants.processorAngle));
		}

    	public void periodic()
    	{
			SmartDashboard.putNumber("Encoder Pos", armEncoder1.getPosition());
    	}

	    //Free move WITH LIMITS (Probably wont use)
    	public Command armUp()
    	{
        	return runOnce(() -> armMotor1.set(1));
    	}

		public Command armDown()
    	{
        	return runOnce(() -> armMotor1.set(-1));
    	}

		public Command armStop()
    	{
        	return runOnce(() -> armMotor1.set(0));
    	}
}