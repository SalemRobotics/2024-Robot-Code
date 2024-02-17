package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;

public class RunShooterIndex extends ParallelCommandGroup {
    final Indexer mIndexer;
    
    public RunShooterIndex(Indexer indexer) {
        mIndexer = indexer;

        addCommands(
            mIndexer.runShooterIndexer(IndexerConstants.kIndexerSpeedIn)
        );

        addRequirements(mIndexer);
    }
}
