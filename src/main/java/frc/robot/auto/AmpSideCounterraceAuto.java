package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;


public class AmpSideCounterraceAuto extends ParallelCommandGroup{
    final Drivetrain mDrivetrain;
	final Intake mIntake;
	final Indexer mIndexer;
	final Shooter mShooter;
	final Vision mVision;

    public AmpSideCounterraceAuto(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Vision vision){
        mDrivetrain = drivetrain;
		mIntake		= intake;
		mIndexer	= indexer;
		mShooter	= shooter;
		mVision		= vision;
            
        addCommands(
            mShooter.shootRing(mVision::getDistance),
			mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn),

            new SequentialCommandGroup(

                AutoBuilder.followPath(
					PathPlannerPath.fromPathFile("Amp Side Counterrace Path 1")
				),

                AutoBuilder.followPath(
					PathPlannerPath.fromPathFile("Amp 6 Note Path 6")
				),

                AutoBuilder.followPath(
					PathPlannerPath.fromPathFile("Amp 7 Note Path 7")
				),

                AutoBuilder.followPath(
					PathPlannerPath.fromPathFile("Amp 7 Note Path 8")
				),
                
                AutoBuilder.followPath(
					PathPlannerPath.fromPathFile("Amp Side Race Path 5")
				),

                AutoBuilder.followPath(
					PathPlannerPath.fromPathFile("Amp Side Race Path 6")
				)

            )
        );
        addRequirements(
			mDrivetrain, mIntake, mIndexer, mVision, mShooter
		);
    }
}
