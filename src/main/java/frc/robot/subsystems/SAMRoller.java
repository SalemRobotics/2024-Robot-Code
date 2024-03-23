package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SAMConstants;

public class SAMRoller extends SubsystemBase {
    final TalonFX mIntakeMotor = new TalonFX(SAMConstants.kIntakeMoterID);
    final DigitalInput mBreakbeam = new DigitalInput(SAMConstants.kBreakbeamID);

    boolean mIsBeamBroke = false;

    /**
     * Runs the SAM roller motor.
     * @param speed Speed to run SAM roller motor at.
     * @return runEnd command
     * @see Command
     */
    public Command runRoller(double speed) {
        return new FunctionalCommand(
            () -> {},
            () -> mIntakeMotor.set(speed), 
            isFinished -> mIntakeMotor.stopMotor(),
            () -> false,
            this
        );
    }

    /**
     * Runs the SAM roller motor.
     * @param speed Speed to run SAM roller motor at.
     * @return runEnd command
     * @see Command
     */
    public Command runRoller(double speed, BooleanSupplier endCondition) {
        return new FunctionalCommand(
            () -> {},
            () -> mIntakeMotor.set(speed), 
            isFinished -> mIntakeMotor.stopMotor(),
            endCondition,
            this
        );
    }

    /**
     * Checks if a game piece has fully passed by the break beam sensor
     * @return True if the gamepiece has passed the break beam
     */
    public boolean hasNotePassedBreakbeam() {
        if (!mBreakbeam.get()) {
            mIsBeamBroke = true;
            return false;
        } 
        
        if (mBreakbeam.get() && mIsBeamBroke) {
            mIsBeamBroke = false;
            return true;
        }

        return false;
    }

    public boolean hasNoteHitBreakbeam() {
        return mBreakbeam.get();
    }
}
