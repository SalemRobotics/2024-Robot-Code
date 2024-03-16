package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;


/**
 * Runs the intake and indexer in parallel,intending to fully eject gamepieces from the robot.
 */
public class IntakeOutAndIndex extends ParallelCommandGroup {
    final Intake mIntake;
    final Indexer mIndexer;

    /**
     * Creates a new {@link IntakeOutAndIndex} command group with required subsystems.
     * @param intake Intake subsystem 
     * @param indexer Indexer subsystem
     * @see Intake
     * @see Indexer
     */
    public IntakeOutAndIndex(Intake intake, Indexer indexer) {
        mIntake = intake;
        mIndexer = indexer;

        addCommands(
            mIntake.intakeRing(IntakeConstants.kIntakeSpeedOut),
            mIndexer.runLowerIndexerTerminate(IndexerConstants.kIndexerSpeedOut)
        );

        addRequirements(mIntake, mIndexer);
    }
}
