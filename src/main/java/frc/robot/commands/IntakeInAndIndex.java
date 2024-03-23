package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * Runs the Intake and Indexer together in parallel, intending to fully intake gamepieces into the robot.
 */
public class IntakeInAndIndex extends ParallelRaceGroup {
    final Intake mIntake;
    final Indexer mIndexer;

    /**
     * Creates a new {@link IntakeInAndIndex} command group with required subsystems.
     * @param intake Intake subsystem 
     * @param indexer Indexer subsystem
     * @see Intake
     * @see Indexer
     */
    public IntakeInAndIndex(Intake intake, Indexer indexer) {
        mIntake = intake;
        mIndexer = indexer;

        addCommands(
            mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn),
            mIndexer.runAllIndexer(IndexerConstants.kIndexerSpeedIn, IndexerConstants.kIndexerSpeedOut)
        );

        addRequirements(mIntake, mIndexer);
    }

    /**
     * Creates a new {@link IntakeInAndIndex} command group with required subsystems and indexer speeds.
     * @param intake Intake subsystem 
     * @param indexer Indexer subsystem
     * @param lowerIndexerSpeed speed to run the lower indexer at(in or out)
     * @param upperIndexerSpeed speed to run the upper indexer at(in or out)
     * @see Intake
     * @see Indexer
     */
    public IntakeInAndIndex(Intake intake, Indexer indexer, double lowerIndexerSpeed, double upperIndexerSpeed) {
        mIntake = intake;
        mIndexer = indexer;
        // TODO: make terminate
        addCommands(
            mIntake.intakeRing(IntakeConstants.kIntakeSpeedIn),
            mIndexer.runAllIndexer(lowerIndexerSpeed, upperIndexerSpeed)
        );

        addRequirements(mIntake, mIndexer);
    }
}
