package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class LobNote extends ParallelCommandGroup {
    final Indexer mIndexer;
    final Shooter mShooter;

    public LobNote(Indexer indexer, Shooter shooter) {
        mIndexer = indexer;
        mShooter = shooter;

        addCommands(
            mShooter.lobRing(),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                mIndexer.runAllIndexer(IndexerConstants.kIndexerSpeedIn)
            )
        );

        addRequirements(mIndexer, mShooter);
    }
}
