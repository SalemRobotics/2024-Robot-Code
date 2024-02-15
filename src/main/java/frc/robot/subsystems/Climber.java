package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    final TalonFX climberMotor1 = new TalonFX(ClimberConstants.kClimberMotor1CanID);
    final TalonFX climberMotor2 = new TalonFX(ClimberConstants.kClimberMotor2CanID);

    public Climber() {
        climberMotor1.setNeutralMode(NeutralModeValue.Brake);

        climberMotor2.setControl(new Follower(ClimberConstants.kClimberMotor1CanID, false));
        climberMotor2.setNeutralMode(NeutralModeValue.Brake);
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
