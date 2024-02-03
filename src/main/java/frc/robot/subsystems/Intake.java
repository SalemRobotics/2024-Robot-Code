package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake subsystem with one motor. Brings gamepieces from the field to within the robot.
 */
public class Intake extends SubsystemBase {
    final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kSparkMaxID, MotorType.kBrushless);

    /**
     * Runs the intake motor.
     * @param speed Speed to run intake motor at.
     * @return runEnd command
     * @see Command
     */
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
