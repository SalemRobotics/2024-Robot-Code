package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kSparkMaxID, MotorType.kBrushless);

    public Command intakeRing(double speed) {
        return runEnd(
            () -> {
                intakeMotor.set(speed);
            }, 
            () -> {
                intakeMotor.set(0);
            }
        );
    }
}
