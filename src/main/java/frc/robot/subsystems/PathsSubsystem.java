package frc.robot.subsystems;

import java.sql.Driver;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathsSubsystem extends SubsystemBase {

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
