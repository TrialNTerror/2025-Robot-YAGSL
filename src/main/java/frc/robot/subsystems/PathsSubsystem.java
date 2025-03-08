package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
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
}
