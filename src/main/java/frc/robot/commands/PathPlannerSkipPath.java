package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Vision;

/** A path planner path that skips and pathfinds to the next provided path should its vision target no longer exist */
public class PathPlannerSkipPath extends ConditionalCommand {
    public PathPlannerSkipPath(String path, String backupPath, Vision vision) {
        super(
            AutoBuilder.followPath(
                PathPlannerPath.fromPathFile(path)
            ), 
			AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile(backupPath), 
				new PathConstraints(
					DriveConstants.kMaxSpeedMetersPerSecond * 0.8, 
					DriveConstants.kMaxAccelerationMetersPerSecond * 0.8, 
					DriveConstants.kMaxAngularSpeedRadiansPerSec, 
					DriveConstants.kMaxAngularAccelerationRadiansPerSec
				)
			),
            vision::hasTarget
        );
    }
	
	public PathPlannerSkipPath(String path, Vision vision) {
		super(
			AutoBuilder.followPath(
                PathPlannerPath.fromPathFile(path)
            ), 
			new InstantCommand(), 
			vision::hasTarget
		);
	}
}
