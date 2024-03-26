package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/**
 * Indexer subsystem with two motors. Manipulates gamepiece when it is inside the robot.
 */
public class Indexer extends SubsystemBase {
    final TalonFX mUpperMotor = new TalonFX(IndexerConstants.kIndexerUpperID);
    final TalonFX mLowerMotor = new TalonFX(IndexerConstants.kIndexerLowerID);

    public Indexer() {
        mUpperMotor.setNeutralMode(NeutralModeValue.Coast);
        
        mLowerMotor.setInverted(true);
        mLowerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Runs the lower indexer motor.
     * @param speed Speed to run lower indexer motor at.
     * @return runEnd command
     * @see Command
     */
    public Command runLowerIndexer(double speed) {
        return runEnd(
            () -> mLowerMotor.set(speed), 
            () -> mLowerMotor.stopMotor()
        );
    }

    /**
     * Runs the upper indexer motor.
     * @param speed Speed to run upper indexer motor at.
     * @return runEnd command
     * @see Command
     */
    public Command runUpperIndexer(double speed) {
        return runEnd(
            () -> mUpperMotor.set(speed), 
            () -> mUpperMotor.stopMotor()
        );
    }

    /**
     * Runs both indexer motors.
     * @param speed Speed to run both indexer motors at.
     */
    private void runAllMotors(double speed) {
        mLowerMotor.set(speed);
        mUpperMotor.set(speed);
    }

    /**
     * Runs both indexer motors in opposite directions.
     * @param lowerIndexerSpeed Speed to run lower indexer motor at.
     * @param upperIndexerSpeed Speed to run upper indexer motor at.
     */
    private void runAllMotors(double lowerIndexerSpeed, double upperIndexerSpeed) {
        mLowerMotor.set(lowerIndexerSpeed);
        mUpperMotor.set(upperIndexerSpeed);
    }

    /**
     * Stops both indexer motors.
     */
    private void stopMotors() {
        mLowerMotor.stopMotor();
        mUpperMotor.stopMotor();
    }

    /**
     * Returns a command that runs both indexer motors and stops 
     * them when it finishes.
     * @param speed Speed to run both indexer motors at.
     * @return runEnd Command
     */
    public Command runAllIndexer(double speed) {
        return runEnd(
            () -> runAllMotors(speed), 
            this::stopMotors
        );
    }

    /**
     * Returns a command that runs both indexer motors in different directions
     * and stops them when it finishes.
     * @param lowerIndexerSpeed Speed to run lower indexer motor at.
     * @param upperIndexerSpeed Speed to run upper indexer motor at.
     * @return runEnd Command
     */
    public Command runAllIndexer(double lowerIndexerSpeed, double upperIndexerSpeed) {
        return runEnd(
            () -> runAllMotors(lowerIndexerSpeed, upperIndexerSpeed), 
            this::stopMotors
        );
    }

    /**
     * Returns a command that runs both indexer motors if canRun 
     * is true and stops them when it finishes.
     * @param speed Speed to run both indexer motors at.
     * @param canRun BooleanSupplier for whether the motors should run
     * @return runEnd Command
     */
    public Command runAllIndexer(double speed, BooleanSupplier canRun) {
        return runEnd(
            () -> {
                if (canRun.getAsBoolean()) runAllMotors(speed);
            }, 
            this::stopMotors
        );
    }
}
