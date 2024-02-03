package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/**
 * Indexer subsystem with two motors. Manipulates gamepiece when it is inside the robot.
 */
public class Indexer extends SubsystemBase {
    final CANSparkMax mIndexIntakeMotor = new CANSparkMax(IndexerConstants.kIndexerIntakeID, MotorType.kBrushless);
    final CANSparkMax mIndexShooterMotor = new CANSparkMax(IndexerConstants.kIndexerShooterID, MotorType.kBrushless);
    
    /**
     * Runs the middle indexer motor only.
     * @param speed Speed to run middle indexer at.
     * @return runEnd command
     * @see Command
     */
    public Command runMiddleIndexer(double speed) {
        return runEnd(
            () -> {
                mIndexIntakeMotor.set(speed);
            },
            () -> {
                mIndexIntakeMotor.stopMotor();
            }
        );
    }

    /**
     * Runs both indexer motors at once.
     * @param speed Speed to run indexer at.
     * @return runEnd command
     * @see Command
     */
    public Command runAllIndexer(double speed) {
        return runEnd(
            () -> {
                mIndexIntakeMotor.set(speed);
                mIndexShooterMotor.set(speed);
            }, 
            () -> {
                mIndexIntakeMotor.stopMotor();
                mIndexShooterMotor.stopMotor();
            }
        );
    }

    /**
     * Runs both indexer motors conditionally.
     * @param speed Speed to run indexer at.
     * @param canRun Supplier that determines when the command can run.
     * @return runEnd command
     * @see Command
     * @see BooleanSupplier
     */
    public Command runAllIndexer(double speed, BooleanSupplier canRun) {
        return runEnd(
            () -> {
                if (canRun.getAsBoolean()) {
                    mIndexIntakeMotor.set(speed);
                    mIndexShooterMotor.set(speed);
                }
            }, 
            () -> {
                mIndexIntakeMotor.stopMotor();
                mIndexShooterMotor.stopMotor();
            }
        );
    }
}
