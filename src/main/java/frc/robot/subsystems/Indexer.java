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
        mUpperMotor.setInverted(true);
        mUpperMotor.setNeutralMode(NeutralModeValue.Coast);
        
        mLowerMotor.setInverted(true);
        mLowerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public Command runLowerIndexer(double speed) {
        return runEnd(
            () -> {
                mLowerMotor.set(speed);
            }, 
            () -> {
                mLowerMotor.stopMotor();
            }
        );
    }

    public Command runUpperIndexer(double speed) {
        return runEnd(
            () -> {
                mUpperMotor.set(speed);
            }, 
            () -> {
                mUpperMotor.stopMotor();
            }
        );
    }

    void runAllMotors(double speed) {
        mLowerMotor.set(speed);
        mUpperMotor.set(speed);
    }

    void stopMotors() {
        mLowerMotor.stopMotor();
        mUpperMotor.stopMotor();
    }

    public Command runAllIndexer(double speed) {
        return runEnd(
            () -> runAllMotors(speed), 
            this::stopMotors
        );
    }

    public Command runAllIndexer(double speed, BooleanSupplier canRun) {
        return runEnd(
            () -> {
                if (canRun.getAsBoolean()) runAllMotors(speed);
            }, 
            this::stopMotors
        );
    }
}
