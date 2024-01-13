package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    CANSparkMax intakeSparkMax = new CANSparkMax(IntakeConstants.kIntakeSparkID, MotorType.kBrushless);

    public Command intakeCube(double speed){
        return run(
            () -> {
                  intakeSparkMax.set(speed);  
            }
        );
    }
}
