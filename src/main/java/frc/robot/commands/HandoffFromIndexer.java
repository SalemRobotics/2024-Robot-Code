package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SAMConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SAMRoller;
import frc.robot.subsystems.SourceAmpMech;
import frc.robot.subsystems.SourceAmpMech.SAMPositions;

public class HandoffFromIndexer extends SequentialCommandGroup {
    final SourceAmpMech mSourceAmpMech;
    final SAMRoller mSamRoller;
    final Intake mIntake;
    final Indexer mIndexer;

    public HandoffFromIndexer(SourceAmpMech sam, SAMRoller roller, Intake intake, Indexer indexer) {
        mSourceAmpMech = sam;
        mSamRoller = roller;
        mIntake = intake;
        mIndexer = indexer;

        addCommands(
            // set SAM to HANDOFF_NOTE position
            mSourceAmpMech.runSAM(SAMPositions.HANDOFF_NOTE),
            new SequentialCommandGroup(
                // run intake forwards, indexer backwards, and SAM roller forwards 
                // until note passes break beam
                new ParallelRaceGroup(
                    mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn),
                    mIndexer.runLowerIndexer(IndexerConstants.kIndexerSpeedOut),
                    mSamRoller.runRoller(SAMConstants.SAMspeedIn, mSamRoller::hasNotePassedBreakbeam)
                ),
                // run SAM roller back a little bit to prevent note from slipping out
                new ParallelDeadlineGroup(
                    new WaitCommand(0.3),
                    mSamRoller.runRoller(SAMConstants.SAMspeedOut) 
                )
            ),
            // set SAM to EJECT_AMP position until button is let go
            mSourceAmpMech.runSAM(SAMPositions.EJECT_AMP)
        );

        addRequirements(mSourceAmpMech, mSamRoller, mIntake, mIndexer);
    }
}
