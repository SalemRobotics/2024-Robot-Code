package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StrongArmMachine;

public class SourceIntakeAndIndex extends SequentialCommandGroup {
    final Intake mIntake;
    final Indexer mIndexer;
    final StrongArmMachine mSAM;

    public SourceIntakeAndIndex(Intake intake, Indexer indexer, StrongArmMachine sam) {
        mIntake = intake;
        mIndexer = indexer;
        mSAM = sam;

        addCommands(
            mSAM.intakeSource(),
            new ParallelRaceGroup(
                mSAM.handoffToIndexer(),
                mIndexer.runLowerIndexerTerminate(IndexerConstants.kIndexerSpeedIn),
                mIntake.intakeRing(IntakeConstants.kIntakeSpeedOut)
            )
        );

        addRequirements(mIntake, mIndexer, mSAM);
    }
}
