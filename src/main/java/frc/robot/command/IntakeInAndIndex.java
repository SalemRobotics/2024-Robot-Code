package frc.robot.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeInAndIndex extends ParallelCommandGroup {
    final Intake mIntake;
    final Indexer mIndexer;

    public IntakeInAndIndex(Intake intake, Indexer indexer) {
        mIntake = intake;
        mIndexer = indexer;
        addRequirements(mIntake, mIndexer);

        addCommands(
            mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn),
            mIndexer.runMiddleIndexer(IndexerConstants.kIndexerSpeedIn)
        );
    }
}
