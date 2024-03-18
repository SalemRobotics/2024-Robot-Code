package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StrongArmMachine;

public class PrepAmp extends SequentialCommandGroup {
    final StrongArmMachine mSAM;
    final Indexer mIndexer;
    final Intake mIntake;
    
    /**
     * Hands off a note into the SAM, and then positions at the AMP height to score
     * @param intake
     * @param indexer
     * @param sam
     */
    public PrepAmp(Intake intake, Indexer indexer, StrongArmMachine sam) {
        mIntake = intake;
        mIndexer = indexer;
        mSAM = sam;

        addCommands(
            new ParallelRaceGroup(
                mSAM.handoffFromIndexer(),
                mIndexer.runLowerIndexer(IndexerConstants.kIndexerSpeedOut),
                mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn)
            ),
            mSAM.holdAtAmp()
        );

        addRequirements(mIntake, mIndexer, mSAM);
    }
}
