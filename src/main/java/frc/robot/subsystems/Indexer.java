package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    final CANSparkMax mIndexShooterMotor = new CANSparkMax(0,MotorType.kBrushless);
    final CANSparkMax mIndexIntakeMotor = new CANSparkMax(0,MotorType.kBrushless);

    public Command primeIndexer(){
        return run(
            () -> {
                mIndexIntakeMotor.set(0.5);
                mIndexShooterMotor.set(-0.5);
            }
        );
    }

    public Command runIndexer(double speed){
        return run(
            () -> {
                mIndexShooterMotor.set(speed);
            }
        );

    }
}
