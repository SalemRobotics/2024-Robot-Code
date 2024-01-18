package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax intakeSparkMaxOne = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax intakeSparkMaxTwo = new CANSparkMax(0, MotorType.kBrushless);

    public Intake() {
        intakeSparkMaxTwo.follow(intakeSparkMaxOne);
    }

    public Command intakeRing(double speed) {
        return run(
            () -> {
                intakeSparkMaxOne.set(speed);
            }
        );
    }
}
