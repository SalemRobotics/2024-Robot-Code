package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    final TalonFX mLeftMotor = new TalonFX(ClimberConstants.kClimberMotor1CanID);
    final TalonFX mRightMotor = new TalonFX(ClimberConstants.kClimberMotor2CanID);

    public Climber() {
        mLeftMotor.setNeutralMode(NeutralModeValue.Brake);

        mRightMotor.setControl(
            new Follower(ClimberConstants.kClimberMotor1CanID, false)
        );
        mRightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command climbTime(double speed) {
        return runEnd(
            () -> {
                mLeftMotor.set(speed);
            }, 
            () -> {
                mLeftMotor.stopMotor();
            }
        );
    }
}
