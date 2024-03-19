package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StrongArmMachine;

public class ScoreAmpFromIndexer extends SequentialCommandGroup {
    final Intake mIntake;
    final Indexer mIndexer;
    final StrongArmMachine mSAM;

    public ScoreAmpFromIndexer(Intake intake, Indexer indexer, StrongArmMachine sam) {
        mIntake = intake;
        mIndexer = indexer;
        mSAM = sam;

        addCommands(
            new ParallelRaceGroup(
                mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn),
                mIndexer.runLowerIndexer(IndexerConstants.kIndexerSpeedOut),
                mSAM.handoffFromIndexer()
            ),
            new ParallelRaceGroup(
                mSAM.eject(-1.0),
                new WaitCommand(.3)
            )
        );

        addRequirements(mIntake, mIndexer, mSAM);
    }
}
