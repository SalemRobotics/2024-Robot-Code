package frc.robot.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class SpinUpShooterAndIndex extends ParallelCommandGroup {
    final Indexer mIndexer;
    final Shooter mShooter;

    public SpinUpShooterAndIndex(Indexer indexer, Shooter shooter) {
        mIndexer = indexer;
        mShooter = shooter;
        addRequirements(mIndexer,mShooter);

        addCommands(
            mShooter.shootRing(),
            new SequentialCommandGroup(
                new WaitCommand(2),
                mIndexer.runMiddleIndexer(IndexerConstants.kIndexerSpeedIn)
            )
        );
    }
}
