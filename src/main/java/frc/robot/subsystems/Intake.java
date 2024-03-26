package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake subsystem with one motor. Brings gamepieces from the field to within the robot.
 */
public class Intake extends SubsystemBase {
    final TalonFX mIntakeMotor = new TalonFX(IntakeConstants.kSparkMaxID);
    final DigitalInput mBreakbeam = new DigitalInput(IntakeConstants.kBreakbeamID);

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Has Note", hasHitBreakbeam());
    }

    public boolean hasHitBreakbeam() {
        return !mBreakbeam.get();
    }

    /**
     * Runs the intake motor.
     * @param speed Speed to run intake motor at.
     * @return runEnd command
     * @see Command
     */
    public Command intakeRing(double speed) {
        return runEnd(
            () -> mIntakeMotor.set(speed), 
            () -> mIntakeMotor.stopMotor()
        );
    }

    /**
     * Runs the intake motor.
     * @param speed Speed to run intake motor at.
     * @param endCondition Boolean function reference to determine when to end the command
     * @return runEnd command
     * @see Command
     */
    public Command intakeRing(double speed, BooleanSupplier endCondition) {
        return new FunctionalCommand(
            () -> {}, // init
            () -> mIntakeMotor.set(speed), // exec
            isFinished -> mIntakeMotor.stopMotor(), // end
            endCondition, // isFinished
            this
        );
    }
}
