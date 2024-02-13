package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    final CANSparkMax climberMotor1 = new CANSparkMax(ClimberConstants.kClimberMotor1CanID, MotorType.kBrushless);
    final CANSparkMax climberMotor2 = new CANSparkMax(ClimberConstants.kClimberMotor2CanID, MotorType.kBrushless);

    public Climber() {
        climberMotor1.setIdleMode(IdleMode.kBrake);
        climberMotor1.burnFlash();

        climberMotor2.follow(climberMotor1);
        climberMotor2.setIdleMode(IdleMode.kBrake);
        climberMotor2.burnFlash();
    }

    public Command climbTime(double speed) {
        return runEnd(
            () -> {
                climberMotor1.set(speed);
            }, 
            () -> {
                climberMotor1.stopMotor();
            }
        );
    }
}
