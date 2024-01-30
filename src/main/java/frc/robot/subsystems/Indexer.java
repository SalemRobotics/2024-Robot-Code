package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    final CANSparkMax mIndexIntakeMotor = new CANSparkMax(IndexerConstants.kIndexerIntakeID, MotorType.kBrushless);
    final CANSparkMax mIndexShooterMotor = new CANSparkMax(IndexerConstants.kIndexerShooterID, MotorType.kBrushless);
    
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

    public Command runShooterIndexer(double speed) {
        return runEnd(
            () -> {
                mIndexShooterMotor.set(speed);
            }, 
            () -> {
                mIndexShooterMotor.stopMotor();
            }
        );
    }
}
