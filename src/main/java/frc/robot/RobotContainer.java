// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.feederHeight;
import frc.robot.subsystems.ElevatorSubsystem.groundHeight;
import frc.robot.subsystems.ElevatorSubsystem.homeHeight;
import frc.robot.subsystems.ElevatorSubsystem.level1Height;
import frc.robot.subsystems.ElevatorSubsystem.level2Height;
import frc.robot.subsystems.ElevatorSubsystem.level3Height;
import frc.robot.subsystems.ElevatorSubsystem.processorHeight;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.feederAngle;
import frc.robot.subsystems.ArmSubsystem.groundAngle;
import frc.robot.subsystems.ArmSubsystem.homeAngle;
import frc.robot.subsystems.ArmSubsystem.level1Angle;
import frc.robot.subsystems.ArmSubsystem.level2Angle;
import frc.robot.subsystems.ArmSubsystem.level3Angle;
import frc.robot.subsystems.ArmSubsystem.processorAngle;
import frc.robot.subsystems.PathsSubsystem;

import java.io.File;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(2);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  //Setup Subsystems
  private final ElevatorSubsystem elevator = new ElevatorSubsystem(); 
  private final HandSubsystem hand = new HandSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final PathsSubsystem paths = new PathsSubsystem();



     //Add commands
  //Arm commands
  groundAngle groundAngle = arm.new groundAngle();
  homeAngle homeAngle = arm.new homeAngle();
  feederAngle feederAngle = arm.new feederAngle();
  processorAngle processorAngle = arm.new processorAngle();

  level3Angle level3Angle = arm.new level3Angle();
  level2Angle level2Angle = arm.new level2Angle();
  level1Angle level1Angle = arm.new level1Angle();

  //Elevator commands
  groundHeight groundHeight = elevator.new groundHeight();
  homeHeight homeHeight = elevator.new homeHeight();
  feederHeight feederHeight = elevator.new feederHeight();
  processorHeight processorHeight = elevator.new processorHeight();

  level3Height level3Height = elevator.new level3Height();
  level2Height level2Height = elevator.new level2Height();
  level1Height level1Height = elevator.new level1Height();



  //set up auto chooser                                                                              
  private final SendableChooser<Command> autoChooser;

  private int currentNum;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * 1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    currentNum = 1;

    SmartDashboard.putData("TEST AUTO", new PathPlannerAuto("TEST AUTO"));

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
     


         //          TESTING


         //Coral Output - operator - TESTING
         operatorXbox.povDown().whileTrue(elevator.elevatorDown()).whileFalse(elevator.elevatorStop());

         operatorXbox.povUp().whileTrue(elevator.elevatorUp()).whileFalse(elevator.elevatorStop());


         driverXbox.a().onTrue(paths.driveToTEST());

     
   
         //    ^^^   TESTING   ^^^

        

          //MAIN DRIVE COMMAND - DRIVER

       //sets driving mode - driver
       drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);



         //SWITCH SCORING SIDE - OPERATOR

       //Switch from front to back scoring - operator
       operatorXbox.start().onTrue(arm.switchScore());



           //HAND COMMANDS - OPERATOR

         //CORAL COMMANDS

       //Coral Intake - operator
       operatorXbox.leftTrigger().whileTrue(hand.intakeCoral()).whileFalse(hand.motorsOff());

       //Coral Output - operator
       operatorXbox.leftBumper().whileTrue(hand.OutputCoral()).whileFalse(hand.motorsOff());

         //ALGAE COMMANDS

       //Algae Intake - operator
       operatorXbox.rightTrigger().whileTrue(hand.intakeAlgae()).whileFalse(hand.motorsOff());
 
       //Algae Output - operator
       operatorXbox.rightBumper().whileTrue(hand.outputAlgae()).whileFalse(hand.motorsOff());
 


         //MISC POSITION COMMANDS
         
       //Home position command - operator       
       operatorXbox.a().onTrue(homeAngle.andThen(homeHeight));

       //Ground position command - operator    
 //      operatorXbox.b().onTrue(groundAngle.andThen(groundHeight));         

       //processor position command - operator
 //      operatorXbox.y().onTrue(processorAngle.andThen(processorHeight));

       //Feeder porsition command - operator
       operatorXbox.x().onTrue(feederAngle.andThen(feederHeight));



         //LEVEL COMMANDS

       //Level 3 position command - operator
       operatorXbox.povUp().onTrue(level3Angle.andThen(level3Height));

       //Level 2 position command - operator
       operatorXbox.povLeft().onTrue(level2Angle.andThen(level2Height));

       //level 1 position command - operator
       operatorXbox.povDown().onTrue(level1Angle.andThen(level1Height));



         //CLIMB LOCK/UNLOCK COMMANDS - DRIVER

       //Lock elevator - driver
//       driverXbox.rightTrigger().onTrue(elevator.lockElevator());

       //Unlock elevator - driver
//       driverXbox.leftTrigger().onTrue(elevator.unlockElevator());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
