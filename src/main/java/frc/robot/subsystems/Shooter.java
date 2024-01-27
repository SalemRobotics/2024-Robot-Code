package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterContants;

public class Shooter extends SubsystemBase {
    final CANSparkMax mLeftMotor = new CANSparkMax(ShooterContants.kLeftMotorID, MotorType.kBrushless);
    final CANSparkMax mRightMotor = new CANSparkMax(ShooterContants.kRightMotorID, MotorType.kBrushless);

    public Shooter() {
        mRightMotor.follow(mLeftMotor, true);
    }

    public Command shootRing() {
        return runEnd(
            () -> {
                mLeftMotor.set(ShooterContants.kShooterSpeed);
            },
            () -> {
                mLeftMotor.stopMotor();
            }
        );
    }
}
