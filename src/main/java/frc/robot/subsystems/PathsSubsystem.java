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
import swervelib.SwerveDrive;



public class PathsSubsystem extends SubsystemBase {

		public Command driveToProcessor() 
		{
			try{
				PathPlannerPath path;
				var alliance = DriverStation.getAlliance();

				if(alliance.isPresent())
				{
					if(alliance.get() == DriverStation.Alliance.Red)
						{
							path = PathPlannerPath.fromPathFile("WP - Processor - Red");
							return AutoBuilder.followPath(path);
						}
						else
						{
							path = PathPlannerPath.fromPathFile("WP - Processor - Blue");
							return AutoBuilder.followPath(path);
						}
				}

				return Commands.none();

				} catch (Exception e) {
					DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
					return Commands.none();
				}
 	    }

		public Command driveToRightFeeder(String side)     //when looking at the pose in robot container, it will assign side either top or bottom depending on Y position
		{
			try{
				var alliance = DriverStation.getAlliance();

				if (alliance.isPresent())
				{
				  if(alliance.get() == DriverStation.Alliance.Red)
				  {
						PathPlannerPath path = PathPlannerPath.fromPathFile("WP - RightFeeder - Red");
						return AutoBuilder.followPath(path);
				  }
				  else
				  {
						PathPlannerPath path = PathPlannerPath.fromPathFile("WP - RightFeeder - Blue");
						return AutoBuilder.followPath(path);
				  }
				}

				return Commands.none();

			} catch (Exception e) {
				DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
				return Commands.none();
			}
		}



		public Command driveToLeftFeeder(String side)     //when looking at the pose in robot container, it will assign side either top or bottom depending on Y position
		{
			try{
				var alliance = DriverStation.getAlliance();

				if (alliance.isPresent())
				{
				  if(alliance.get() == DriverStation.Alliance.Red)
				  {
						PathPlannerPath path = PathPlannerPath.fromPathFile("WP - LeftFeeder - Red");
						return AutoBuilder.followPath(path);
				  }
				  else
				  {
						PathPlannerPath path = PathPlannerPath.fromPathFile("WP - LeftFeeder - Blue");
						return AutoBuilder.followPath(path);
				  }
				}

				return Commands.none();

			} catch (Exception e) {
				DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
				return Commands.none();
			}
		}



		public Command driveToTEST() 
		{
			try{
				PathPlannerPath path;
				var alliance = DriverStation.getAlliance();

				if(alliance.isPresent())
				{
					if(alliance.get() == DriverStation.Alliance.Red)
						{
							path = PathPlannerPath.fromPathFile("TEST PATH");    //Is for blue alliance
							return AutoBuilder.followPath(path);
						}
						else
						{
							path = PathPlannerPath.fromPathFile("TEST PATH");
							return AutoBuilder.followPath(path);
						}
				}

				return Commands.none();

				} catch (Exception e) {
					DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
					return Commands.none();
				}
 	    }


}

