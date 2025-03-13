package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import edu.wpi.first.math.util.Units;

public class PathsSubsystem extends SubsystemBase {

	//private final Vision vision = new Vision(null, null);
	//private final 

		public Command testPos() 
		{
			try{
					// Load the path you want to follow using its name in the GUI
					PathPlannerPath path = PathPlannerPath.fromPathFile("1stScore");

					// Create a path following command using AutoBuilder. This will also trigger event markers.
					return AutoBuilder.followPath(path);
				} catch (Exception e) {
					DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
					return Commands.none();
				}
 	    }

		public Command BlueReefApos(SwerveSubsystem drivebase)
		{
			// Add a button to SmartDashboard that will create and follow an on-the-fly path
			// This example will simply move the robot 2m in the +X field direction
			Commands.runOnce(() -> {
			Pose2d currentPose = drivebase.getPose();
			
			// The rotation component in these poses represents the direction of travel
			Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
			Pose2d endPos = Constants.PoseConstants.BlueReefAPose;

			List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
			PathPlannerPath path = new PathPlannerPath(
				waypoints, 
				new PathConstraints(
				4.0, 4.0, 
				Units.degreesToRadians(360), Units.degreesToRadians(540)
				),
				null, // Ideal starting state can be null for on-the-fly paths
				new GoalEndState(0.0, currentPose.getRotation())
			);

			// Prevent this path from being flipped on the red alliance, since the given positions are already correct
			path.preventFlipping = true;

			AutoBuilder.followPath(path).schedule();
			})
		}

		public Command GotoFeeder(Pose2d currentPose){
			try{
				var alliance = DriverStation.getAlliance();
				Pose2d feederstationpose = new Pose2d();
				if (alliance.isPresent())
				{
				  if(alliance.get() == DriverStation.Alliance.Red){
					if(currentPose.getY() > 12){
						//Go to Red Right Station
					}
					else{
						//Go to Red Left Station
					}
				  }
				  else{
					if(currentPose.getY() < 12){
						//Go to Blue Right Station
					}
					else{
						//Go to Blue Left Station
					}
				  }
				}
				else{
					return Commands.none();
				}

				return AutoBuilder.pathfindToPose(null, null, 0);

			} catch (Exception e) {
				DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
				return Commands.none();
			}
		}

}
