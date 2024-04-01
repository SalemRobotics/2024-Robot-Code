package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.PathPlannerSkipPath;
import frc.robot.commands.SpinUpShooterAndIndex;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class NonAmpRaceAuto extends ParallelCommandGroup {
	final Drivetrain mDrivetrain;
	final Intake mIntake;
	final Indexer mIndexer;
	final Shooter mShooter;
	final Vision mVision;

	public NonAmpRaceAuto(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Vision vision) {
		mDrivetrain = drivetrain;
		mIntake		= intake;
		mIndexer	= indexer;
		mShooter	= shooter;
		mVision		= vision;

		addCommands(
			mShooter.shootRing(mVision::getDistance),
			mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn),

			new SequentialCommandGroup(
				
				new ParallelDeadlineGroup(
					new WaitCommand(1.0), 
					new SpinUpShooterAndIndex(indexer, shooter, vision)	
				),

				new PathPlannerSkipPath(
					"Copy of Non-Amp Race Path 1", 
					"Non-Amp 6 Note Path 6", 
					vision
				),
				
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.5), 
						new ParallelDeadlineGroup(
							new WaitCommand(1.5), 
							mIndexer.runAllIndexer(IndexerConstants.kIndexerSpeedIn)
						)
					),
					mDrivetrain.trackTarget(mVision::getDistance, mVision::getYaw, () -> 0, () -> 0)
				),

				AutoBuilder.followPath(
					PathPlannerPath.fromPathFile("Non-Amp 6 Note Path 6")
				),

				new PathPlannerSkipPath(
					"Non-Amp 6 Note Path 6", 
					"Non-Amp 6 Note Path 7", 
					vision
				),

				new PathPlannerSkipPath(
					"Non-Amp 6 Note Path 7",
					vision
				),

				new ParallelCommandGroup(
					new SequentialCommandGroup(
						new WaitCommand(1.0),
						mIndexer.runAllIndexer(IndexerConstants.kIndexerSpeedIn)
					),
					mDrivetrain.trackTarget(mVision::getDistance, mVision::getYaw, () -> 0, () -> 0)
				)
			)
		);
		
		addRequirements(
			mDrivetrain, mIntake, mIndexer, mVision, mShooter
		);
	}
}
