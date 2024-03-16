package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * Runs the shooter and indexer together, 
 * intending to speed the shooter up to an allowed threshold before running the indexer to fire gamepieces. 
 */
public class TrackTargetAndShoot extends ParallelCommandGroup {
    final Drivetrain mDrivetrain;
    final Vision mVision;
    final Indexer mIndexer;
    final Shooter mShooter;

    /**
     * Creates a new {@link TrackTargetAndShoot} command group with required subsystems.
     * This command will aim the robot at the target and shoot at it.
     * @param drivetrain Drivetrain subsystem
     * @param vision Vision subsystem
     * @param indexer Indexer subsystem
     * @param shooter Shooter subsystem
     * @param xSpeed X speed (forward/reverse) for tracking the target
     * @param ySpeed Y speed (left/right) for tracking the target
     * @see Indexer
     * @see Shooter
     */
    public TrackTargetAndShoot(Drivetrain drivetrain, Vision vision, Indexer indexer, Shooter shooter, double xSpeed, double ySpeed) {
        mDrivetrain = drivetrain;
        mVision = vision;
        mShooter = shooter;
        mIndexer = indexer;
        
        addCommands(
            mDrivetrain.trackTarget(()-> mVision.getYaw(), xSpeed, ySpeed),
            mShooter.shootRing(mVision::getDistance),
            mIndexer.runAllIndexer(IndexerConstants.kIndexerSpeedIn, mShooter::atOutputThreshold)
        );

        addRequirements(mDrivetrain, mVision, mIndexer, mShooter);
    }
}
