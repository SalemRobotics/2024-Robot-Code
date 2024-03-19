package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SAMConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SAMRoller;
import frc.robot.subsystems.SourceAmpMech;
import frc.robot.subsystems.SourceAmpMech.SAMPositions;

public class HandoffToIndexer extends SequentialCommandGroup {
    final SourceAmpMech mSourceAmpMech;
    final SAMRoller mSamRoller;
    final Intake mIntake;
    final Indexer mIndexer;

    public HandoffToIndexer(SourceAmpMech sam, SAMRoller roller, Intake intake, Indexer indexer) {
        mSourceAmpMech = sam;
        mSamRoller = roller;
        mIntake = intake;
        mIndexer = indexer;

        addCommands(
            // set SAM to HANDOFF_NOTE position
            mSourceAmpMech.runSAM(SAMPositions.HANDOFF_NOTE),
            // run intake backwards, indexer forwards, and SAM roller backwards 
            new ParallelRaceGroup(
                mIntake.intakeRing(IntakeConstants.kIntakeSpeedOut),
                mIndexer.runLowerIndexer(IndexerConstants.kIndexerSpeedIn),
                mSamRoller.runRoller(SAMConstants.SAMspeedOut)
            )
        );

        addRequirements(mSourceAmpMech, mSamRoller, mIntake, mIndexer);
    }
}
