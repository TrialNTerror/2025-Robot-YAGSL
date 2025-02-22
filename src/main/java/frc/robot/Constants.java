// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants
  {
    //CAN IDs
    public static final int elevatorLeadMotorCanID = 11;        //ADD MARKING ON MOTOR AS TO WHICH IS WHICH
    public static final int elevatorFollowMotor2CanID = 12;
    public static final int servoIDLeadSide = 0;
    public static final int servoIDFollowSide = 1;

    //Min/Max
    public static final double minOutputElevator = 0.0;
    public static final double maxOutputElevator = 1.0;

    //Feedforward
    public static final double kStaticGain = 0.0;
    public static final double kGravity = 0.0;
    public static final double kVelocity = 0.0;
    public static final double kAccel = 0.0;

    //Positions
    public static final double level3Height = 0.75;
    public static final double level2Height = 0.75;
    public static final double level1Height = 0.75;
    public static final double groundHeight = 0.75;
    public static final double homeHeight = 0.1;
    public static final double processorHeight = 0;

    //Lock/unlock Servo
    public static final int servoFollowLock = 55;
    public static final int servoFollowUnlock = 70;
    public static final int servoLeaderLock = 0;
    public static final int servoLeaderUnlock = 25;

    //Motor Inversion
    public static final boolean leadMotorInverted = false;
    public static final boolean followMotorInverted = false;

    //motor limits
    public static final int stallLimit = 40;
    public static final double maxAcceleration = 36000;
    public static final double maxVelocity = 360;
    public static final double allowedErr = 0.5;
    
    //motor config
    public static final double P = 20;
    public static final double I = 0;
    public static final double D = 0.1;
    public static final boolean inverted = true;

    //motor conversion
    public static final double positionConversionFactor = 360;
    public static final double velocityConversionFactor = 360;
  }

  public static class ArmConstants
  {
    //Constants for feedforward
    public static final double kStaticGain = 0.0;
    public static final double kGravity = 0.0;
    public static final double kVelocity = 0.0;
    public static final double kAccel = 0.0;

    //CAN IDs
    public static final int armMotor1CanID = 13;

    //Min/Max output
    public static final double minOutputArm = -1.0;
    public static final double maxOutputArm = 1.0;

    //Positions
    public static final double level3Angle = 0.75;
    public static final double level3BackAngle = -0.75;

    public static final double level2Angle = 0.75;
    public static final double level2BackAngle = -0.75;

    public static final double level1Angle = 0.75;
    public static final double level1BackAngle = -0.75;

    public static final double groundAngle = 0.2;
    public static final double homeAngle = 0.9;
    public static final double processorAngle = 0;

    //inverting
    public static final boolean motorInvert = false;
    
    //relative min and max arm angles
    public static final double maxAngle = 180;
    public static final double minAngle = 0;

    public static final int stallLimit = 40;
    public static final double P = 20;
    public static final double I = 0;
    public static final double D = 0.1;
    public static final double maxAcceleration = 36000;
    public static final double maxVelocity = 360;
    public static final double allowedErr = 0.5;
    public static final boolean inverted = true;
    public static final double positionConversionFactor = 360;
    public static final double velocityConversionFactor = 360;
    //public static final double ;
  }

  public static class HandConstants
  {
    //CAN IDs
    public static final int handMotor1CanID = 14;
    public static final int handMotor2CanID = 15;
    public static final int handMotor3CanID = 16;

    //Intake Speed
    public static final double intakeSpeed = 0.25;

    //Motor inversion
    public static final boolean invertAllMotors = true;    //All motors should be set to inverted but if this is incorrect change this.
  }
}

