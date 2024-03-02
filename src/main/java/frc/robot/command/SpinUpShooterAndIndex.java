package frc.robot.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/**
 * Runs the shooter and indexer together, 
 * intending to speed the shooter up to an allowed threshold before running the indexer to fire gamepieces. 
 */
public class SpinUpShooterAndIndex extends ParallelCommandGroup {
    final Indexer mIndexer;
    final Shooter mShooter;

    /**
     * Creates a new {@link SpinUpShooterAndIndex} command group with required subsystems
     * @param indexer Indexer subsystem
     * @param shooter Shooter subsystem
     * @see Indexer
     * @see Shooter
     */
    public SpinUpShooterAndIndex(Indexer indexer, Shooter shooter) {
        mIndexer = indexer;
        mShooter = shooter;
        
        addCommands(
            mShooter.shootRing(),
            mIndexer.runAllIndexer(IndexerConstants.kIndexerSpeedIn, mShooter::atOutputThreshold)
        );

        addRequirements(mIndexer, mShooter);
    }
}
