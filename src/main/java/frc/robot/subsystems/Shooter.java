package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterContants;

/**
 * Shooter subsystem with two motors. Responsible for firing game pieces from the robot to the speaker goal.
 */
public class Shooter extends SubsystemBase {
    final CANSparkMax mLeftMotor = new CANSparkMax(ShooterContants.kLeftMotorID, MotorType.kBrushless);
    final CANSparkMax mRightMotor = new CANSparkMax(ShooterContants.kRightMotorID, MotorType.kBrushless);

    public Shooter() {
        mRightMotor.follow(mLeftMotor, true);
    }

    /**
     * Determines if the shooter is running within an allowed speed threshold.
     * @return true if within allowed velocity, in RPM
     */
    public boolean isAllowedShootSpeed() {
        return mLeftMotor.getEncoder().getVelocity() >= ShooterContants.kAllowedOutputVelocity;
    }

    /**
     * Runs the shooter at full speed.
     * @return runEnd command
     * @see Command
     */
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
