// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
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

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(2.5, 0, 0);//0.7
    public static final PIDConstants ANGLE_PID       = new PIDConstants(1.5, 0.01);    //0.4 and 0.01
  }

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

    // Min / Max Outouts
    public static final double minOutputElevator = -1.0;
    public static final double maxOutputElevator = 1.0;

    //Feedforward
    public static final double kStaticGain = 0.0;
    public static final double kGravity = 0.0;
    public static final double kVelocity = 0.0;
    public static final double kAccel = 0.0;

    //Positions
    public static final double level3Height = 9000;   //currently testing
    public static final double level2Height = 30;
    public static final double level1Height = 60;   //currently testing

    public static final double groundHeight = 25000;
    public static final double homeHeight = 6000;
    public static final double feederHeight = 80;
    public static final double processorHeight = 293;

    // Lock / Unlock Servo
    public static final int servoFollowLock = 55;
    public static final int servoFollowUnlock = 70;
    public static final int servoLeaderLock = 0;
    public static final int servoLeaderUnlock = 25;

    //Motor Inversion
    public static final boolean leadMotorInverted = false;

    //Encoder Inversion
    public static final boolean inverted = false;

    //Motor Limits
    public static final int currentLimit = 40;
    public static final double maxAcceleration = 36000;
    public static final double maxVelocity = 360;
    public static final double allowedErr = 10;
    
    //Motor Config
    public static final double P = 1;
    public static final double I = 0;
    public static final double D = 0;
    public static final double F = 0;

    //Motor Conversion
    public static final double positionConversionFactor = 360;
    public static final double velocityConversionFactor = 360;

    public static final double encoderAllowError = 1000;
  }



  public static class ArmConstants
  {
    //CAN IDs
    public static final int armMotor1CanID = 13;

    // Min / Max output
    public static final double minOutputArm = -1.0;
    public static final double maxOutputArm = 1.0;

    //Constants for feedforward
    public static final double kStaticGain = 0.0;
    public static final double kGravity = 0.0;
    public static final double kVelocity = 0.0;
    public static final double kAccel = 0.0;

    //Positions
    public static final double level3Angle = 98;     
    public static final double level3BackAngle = 279;

    public static final double level2Angle = 86;
    public static final double level2BackAngle = 287;

    public static final double level1Angle = 291;
    public static final double level1BackAngle = 291;

    public static final double processorFront = 293;
    public static final double processorBack = 115;

    public static final double groundAngle = 300;  //should be at 309
    public static final double homeAngle = 190;
    public static final double feederAngle = 208;

    //Inverting Motor
    public static final boolean motorInvert = false;

    //Inverted Encoder
    public static final boolean inverted = true;
    
    // Min and Max Arm Angles
    public static final double maxAngle = 340;
    public static final double minAngle = 20;

    //Motor Limits
    public static final int currentLimit = 40;
    public static final double maxAcceleration = 36000;
    public static final double maxVelocity = 360;
    public static final double allowedError = 20;

    //Motor Config
    public static final double P = 0.012; //P .02
    public static final double I = 0; // I 0
    public static final double D = 0.0; // D 0.1
    public static final double F = 0;

    //Motor Conversion
    public static final double positionConversionFactor = 360;
    public static final double velocityConversionFactor = 360;

    //LIMITS
    public static final double forwardLimit = 315;  //295
    public static final double reverseLimit = 135;

    public static final double encoderAllowError = 3;
  }



  public static class HandConstants
  {
    //CAN IDs
    public static final int topHandCanID = 14;
    public static final int bottomHandCanID = 15;
    public static final int holdHandCanID = 16;

    //Intake Speed
    public static final double intakeSpeed = 0.5;

    //Smart Current Limit
    public static final int currentLimit = 20;

    //Motor inversion
    public static final boolean invertAllMotors = true;    //All motors should be set to inverted but if this is incorrect change this.
  }

  public static class PoseConstants
  {

    public static double robotSizeOffestX60 = 17.25; //29.5 +5 in (17.25, 29.878)
    public static double robotSizeOffestY60 = 29.878;
    public static double robotPosition1OffestX60 = -4.746;//robot position offset = 5.48 (-4.746, 2.74) 
    public static double robotPosition1OffestY60 = 2.74;
    public static double robotPosition2OffestX60 = 4.746;//robot position offset = 5.48 (-4.746, 2.74) 
    public static double robotPosition2OffestY60 = -2.74;

    public static double robotSizeOffestX120 = -17.25; //29.5 +5 in (17.25, 29.878)
    public static double robotSizeOffestY120 = 29.878;
    public static double robotPosition1OffestX120 = -4.746;//robot position offset = 5.48 (-4.746, 2.74) 
    public static double robotPosition1OffestY120 = 2.74;
    public static double robotPosition2OffestX120 = 4.746;//robot position offset = 5.48 (-4.746, 2.74) 
    public static double robotPosition2OffestY120 = -2.74;

    public static double robotSizeOffestXRedFarthestFrom0CoralFeeder = -20.279;
    public static double robotSizeOffestYRedFarthestFrom0CoralFeeder = 27.911;

    public static final Pose2d BlueLeftFeeder = new Pose2d(Units.inchesToMeters(33.51 + robotSizeOffestXRedFarthestFrom0CoralFeeder), Units.inchesToMeters(291.20 - robotSizeOffestYRedFarthestFrom0CoralFeeder), new Rotation2d(Units.degreesToRadians(306)));
    public static final Pose2d BlueRightFeeder = new Pose2d(Units.inchesToMeters(33.51 - robotSizeOffestXRedFarthestFrom0CoralFeeder), Units.inchesToMeters(25.80 + robotSizeOffestYRedFarthestFrom0CoralFeeder), new Rotation2d(Units.degreesToRadians(54)));

    public static final Pose2d RedLeftFeeder = new Pose2d(Units.inchesToMeters(657.3 - robotSizeOffestXRedFarthestFrom0CoralFeeder), Units.inchesToMeters(25.80 - robotSizeOffestYRedFarthestFrom0CoralFeeder), new Rotation2d(Units.degreesToRadians(126)));
    public static final Pose2d RedRightFeeder = new Pose2d(Units.inchesToMeters(657.37 + robotSizeOffestXRedFarthestFrom0CoralFeeder), Units.inchesToMeters(291.20 + robotSizeOffestYRedFarthestFrom0CoralFeeder), new Rotation2d(Units.degreesToRadians(234)));

    public static final Pose2d RedReefAPose = new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50 - 5.48), new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d RedReefBPose = new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50 + 5.48), new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d RedReefCPose = new Pose2d(Units.inchesToMeters(530.49 + robotSizeOffestX60 + robotPosition1OffestX60), Units.inchesToMeters(186.83 + robotPosition1OffestY60 + robotSizeOffestY60), new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d RedReefDPose = new Pose2d(Units.inchesToMeters(530.49 + robotSizeOffestX60 + robotPosition2OffestX60), Units.inchesToMeters(186.83 + robotPosition2OffestY60 + robotSizeOffestY60), new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d RedReefEPose = new Pose2d(Units.inchesToMeters(497.77 + robotPosition1OffestX120 + robotSizeOffestX120), Units.inchesToMeters(186.83 + robotPosition1OffestY120 + robotSizeOffestY120), new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d RedReefFPose = new Pose2d(Units.inchesToMeters(497.77 + robotPosition2OffestX120 + robotSizeOffestX120), Units.inchesToMeters(497.77 + robotPosition2OffestY120 + robotSizeOffestY120), new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d RedReefGPose = new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50 + 5.48), new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RedReefHPose = new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50 - 5.48), new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RedReefIPose = new Pose2d(Units.inchesToMeters(497.77 + robotPosition1OffestX120 + robotSizeOffestX120), Units.inchesToMeters(130.17 - robotPosition1OffestY120 - robotSizeOffestY120), new Rotation2d(Units.degreesToRadians(240)));
    public static final Pose2d RedReefJPose = new Pose2d(Units.inchesToMeters(497.77 + robotPosition2OffestX120 + robotSizeOffestX120), Units.inchesToMeters(130.17 - robotPosition2OffestY120 - robotSizeOffestY120), new Rotation2d(Units.degreesToRadians(240)));
    public static final Pose2d RedReefKPose = new Pose2d(Units.inchesToMeters(530.49 + robotSizeOffestX60 + robotPosition1OffestX60), Units.inchesToMeters(130.17 - robotPosition1OffestY60 - robotSizeOffestY60), new Rotation2d(Units.degreesToRadians(300)));
    public static final Pose2d RedReefLPose = new Pose2d(Units.inchesToMeters(530.49 + robotSizeOffestX60 + robotPosition2OffestX60), Units.inchesToMeters(130.17 - robotPosition2OffestY60 - robotSizeOffestY60), new Rotation2d(Units.degreesToRadians(300)));

    public static final Pose2d AllRedReefPoses[] = {RedReefAPose, RedReefBPose, RedReefCPose, RedReefDPose, RedReefEPose, RedReefFPose, RedReefGPose, RedReefHPose, RedReefHPose, RedReefIPose, RedReefJPose, RedReefKPose, RedReefLPose};

    public static final Pose2d BlueReefAPose = new Pose2d(Units.inchesToMeters(144), Units.inchesToMeters(158.50 + 5.48), new Rotation2d(180));
    public static final Pose2d BlueReefBPose = new Pose2d(Units.inchesToMeters(144), Units.inchesToMeters(158.50 - 5.48), new Rotation2d(180));
    public static final Pose2d BlueReefCPose = new Pose2d(Units.inchesToMeters(160.39 + robotPosition1OffestX120 + robotSizeOffestX120), Units.inchesToMeters(130.17 - robotPosition1OffestY120 - robotSizeOffestY120), new Rotation2d(240));
    public static final Pose2d BlueReefDPose = new Pose2d(Units.inchesToMeters(160.39 + robotPosition2OffestX120 + robotSizeOffestX120), Units.inchesToMeters(130.17 - robotPosition2OffestY120 - robotSizeOffestY120), new Rotation2d(240));
    public static final Pose2d BlueReefEPose = new Pose2d(Units.inchesToMeters(193.10 + robotSizeOffestX60 + robotPosition1OffestX60), Units.inchesToMeters(130.17 - robotPosition1OffestY60 - robotSizeOffestY60), new Rotation2d(300));
    public static final Pose2d BlueReefFPose = new Pose2d(Units.inchesToMeters(193.10 + robotSizeOffestX60 + robotPosition2OffestX60), Units.inchesToMeters(130.17 - robotPosition2OffestY60 - robotSizeOffestY60), new Rotation2d(300));
    public static final Pose2d BlueReefGPose = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50 - 5.48), new Rotation2d(0));
    public static final Pose2d BlueReefHPose = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50 + 5.48), new Rotation2d(0));
    public static final Pose2d BlueReefIPose = new Pose2d(Units.inchesToMeters(193.10 + robotSizeOffestX60 + robotPosition1OffestX60), Units.inchesToMeters(186.83 + robotPosition1OffestY60 + robotSizeOffestY60), new Rotation2d(60));
    public static final Pose2d BlueReefJPose = new Pose2d(Units.inchesToMeters(193.10 + robotSizeOffestX60 + robotPosition2OffestX60), Units.inchesToMeters(186.83 + robotPosition2OffestY60 + robotSizeOffestY60), new Rotation2d(60));
    public static final Pose2d BlueReefKPose = new Pose2d(Units.inchesToMeters(160.39 + robotPosition1OffestX120 + robotSizeOffestX120), Units.inchesToMeters(186.83 + robotPosition1OffestY60 + robotSizeOffestY60), new Rotation2d(120));
    public static final Pose2d BlueReefLPose = new Pose2d(Units.inchesToMeters(160.39 + robotPosition2OffestX120 + robotSizeOffestX120), Units.inchesToMeters(186.83 + robotPosition2OffestY60 + robotSizeOffestY60), new Rotation2d(120));

    public static final Pose2d AllBlueReefPoses[] = {BlueReefAPose, BlueReefBPose, BlueReefCPose, BlueReefDPose, BlueReefEPose, BlueReefFPose, BlueReefGPose, BlueReefHPose, BlueReefHPose, BlueReefIPose, BlueReefJPose, BlueReefKPose, BlueReefLPose};

  }
}