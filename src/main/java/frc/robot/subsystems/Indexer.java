package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

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

    public Command runAllIndexer(double speed) {
        return runEnd(
            () -> {
                mLowerMotor.set(speed);
                mUpperMotor.set(speed);
            }, 
            () -> {
                mLowerMotor.stopMotor();
                mUpperMotor.stopMotor();
            }
        );
    }
}
