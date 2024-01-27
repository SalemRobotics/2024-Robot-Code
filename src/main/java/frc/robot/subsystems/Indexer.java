package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    final CANSparkMax mIndexIntakeMotor = new CANSparkMax(IndexerConstants.kIndexerIntakeID, MotorType.kBrushless);
    final CANSparkMax mIndexShooterMotor = new CANSparkMax(IndexerConstants.kIndexerShooterID, MotorType.kBrushless);

    public Indexer() {
        mIndexIntakeMotor.setInverted(true);
        mIndexIntakeMotor.burnFlash();

        mIndexShooterMotor.setInverted(true);
        mIndexShooterMotor.burnFlash();
    }

    public Command runIndexer(){
        return runEnd(
            () -> {
                mIndexIntakeMotor.set(IndexerConstants.kIndexerSpeed);
                mIndexShooterMotor.set(IndexerConstants.kIndexerSpeed);
            },
            () -> {
                mIndexIntakeMotor.stopMotor();
                mIndexShooterMotor.stopMotor();
            }
        );

    }
}
