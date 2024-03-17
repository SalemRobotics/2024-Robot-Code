package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * Runs the shooter and indexer together, 
 * intending to speed the shooter up to an allowed threshold before running the indexer to fire gamepieces. 
 */
public class SpinUpShooterAndIndex extends ParallelCommandGroup {
    final Indexer mIndexer;
    final Shooter mShooter;
    final Vision mVision;

    /**
     * Creates a new {@link SpinUpShooterAndIndex} command group with required subsystems
     * @param indexer Indexer subsystem
     * @param shooter Shooter subsystem
     * @see Indexer
     * @see Shooter
     */
    public SpinUpShooterAndIndex(Indexer indexer, Shooter shooter, Vision vision) {
        mIndexer = indexer;
        mShooter = shooter;
        mVision = vision;
        
        addCommands(
            mShooter.shootRing(mVision::getDistance),
            mIndexer.runAllIndexer(IndexerConstants.kIndexerSpeedIn, mShooter::atOutputThreshold)
        );

        addRequirements(mIndexer, mShooter, mVision);
    }
}
