package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StrongArmMachine extends SubsystemBase {
    final CANSparkMax mPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
    final CANSparkMax mIndexMotor = new CANSparkMax(1, MotorType.kBrushless);

    public Command runIndexer(){
        return runEnd(
            () -> {
                
            },
            () -> {
                
            }
        );

    }
}
