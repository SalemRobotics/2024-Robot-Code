package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;

/**
 * Utility to pick autonomous routines in a survey like fashion, similar to 6328's implementation.
 * TODO: Should this be in utils?
 */
public class AutoPicker {
    static final SendableChooser<PathPlannerPath> mStartQuery = new SendableChooser<>();
    static final SendableChooser<PathPlannerPath> mPieceQuery = new SendableChooser<>();
    static final SendableChooser<PathPlannerPath> mEndQuery   = new SendableChooser<>();

    public AutoPicker() {
        for (String option : AutoConstants.kStartOptions) {
            mStartQuery.addOption(option, PathPlannerPath.fromPathFile(option));
        }
        SmartDashboard.putData(mStartQuery);
        
        for (String option : AutoConstants.kPieceOptions) {
            mPieceQuery.addOption(option, PathPlannerPath.fromPathFile(option));
        }
        SmartDashboard.putData(mPieceQuery);

        for (String option : AutoConstants.kEndOptions) {
            mEndQuery.addOption(option, PathPlannerPath.fromPathFile(option));
        }
        SmartDashboard.putData(mEndQuery);
    }

    public static Command buildAuto() {
        var startPath = mStartQuery.getSelected();
        var piecePath = mPieceQuery.getSelected();
        var endPath   = mEndQuery.getSelected();
        
        return new SequentialCommandGroup(
            AutoBuilder.followPath(startPath),
            AutoBuilder.followPath(piecePath),
            AutoBuilder.followPath(endPath)
        );
    }
}
