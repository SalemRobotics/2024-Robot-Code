package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    CANSparkMax climberMoter1 = new CANSparkMax(0,MotorType.kBrushless);
    CANSparkMax climberMoter2 = new CANSparkMax(0,MotorType.kBrushless);

    public Climber() {
        climberMoter2.follow(climberMoter1);
    }

    public Command climbTime(double speed) {
        return runEnd(
            () -> {
                climberMoter1.set(speed);
            }, 
            () -> {
                climberMoter1.set(0);
            }
        );
    }
}
