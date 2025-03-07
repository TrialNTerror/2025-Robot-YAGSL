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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import java.io.File;
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


  private final ElevatorSubsystem elevator = new ElevatorSubsystem(); 
  private final HandSubsystem hand = new HandSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();

  PathPlannerPath path = PathPlannerPath.fromPathFile("2ndScore");

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

    SmartDashboard.putData("ScoreOne", new PathPlannerAuto("ScoreOne"));

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
     
      //Main drive command - driver
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

     // driverXbox.rightTrigger().onTrue(elevator.lockElevator());

     // driverXbox.leftTrigger().onTrue(elevator.unlockElevator());

      //Letter commands (TEST)
        //Home angle
      driverXbox.b().onTrue(arm.homeAngle());

        //ground angle
      driverXbox.x().onTrue(arm.groundAngle());

        //Home angle
      driverXbox.y().onTrue(arm.processorAngle());


      //elevator commands
      driverXbox.povDown().onTrue(elevator.level1Height());

      driverXbox.povUp().onTrue(elevator.level3Height());

       //Level 2 position command  - operator
         ParallelCommandGroup level2Position = 
           new ParallelCommandGroup(elevator.level2Height(), arm.level2Angle());
       driverXbox.povRight().onTrue(level2Position);

/* 
      //DPad commands
        //Level 1 position command 
      driverXbox.povDown().onTrue(arm.level1Angle());

        //Level 2 position command
      driverXbox.povRight().onTrue(arm.level2Angle());
      driverXbox.povLeft().onTrue(arm.level2Angle());

        //Level 3 position command
      driverXbox.povUp().onTrue(arm.level3Angle());

      //switch between front and back scoring 
      driverXbox.start().onTrue(arm.switchScore());
*/
      
     
      //Coral Intake - operator
      driverXbox.rightTrigger().whileTrue(hand.intakeCoral()).whileFalse(hand.motorsOff());

      //Coral Output - operator
      driverXbox.rightBumper().whileTrue(hand.OutputCoral()).whileFalse(hand.motorsOff());

      //Algae Intake - operator
      driverXbox.leftTrigger().whileTrue(hand.intakeAlgae()).whileFalse(hand.motorsOff());

      //Algae Output - operator
      driverXbox.leftBumper().whileTrue(hand.outputAlgae()).whileFalse(hand.motorsOff());

      /* 
      //Home position command - operator
          ParallelCommandGroup homePosition = 
            new ParallelCommandGroup(elevator.homeHeight(), arm.homeAngle());
      operatorXbox.a().onTrue(homePosition);

      //Ground position command - operator
          ParallelCommandGroup groundPosition = 
            new ParallelCommandGroup(elevator.groundHeight(), arm.groundAngle());
      operatorXbox.x().onTrue(groundPosition);
            
      //processor position command  - operator
         ParallelCommandGroup processorPosition = 
            new ParallelCommandGroup(elevator.processorHeight(), arm.processorAngle());
      operatorXbox.y().onTrue(processorPosition);


      //Level 1 position command - operator
          ParallelCommandGroup level1Position = 
            new ParallelCommandGroup(elevator.level1Height(), arm.level1Angle());
      operatorXbox.povDown().onTrue(level1Position);

      //Level 2 position command  - operator
          ParallelCommandGroup level2Position = 
            new ParallelCommandGroup(elevator.level2Height(), arm.level2Angle());
      operatorXbox.povLeft().onTrue(level2Position);

      //level 3 position command  - operator
          ParallelCommandGroup level3Position = 
            new ParallelCommandGroup(elevator.level3Height(), arm.level3Angle());
      operatorXbox.povUp().onTrue(level3Position);
      */
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
