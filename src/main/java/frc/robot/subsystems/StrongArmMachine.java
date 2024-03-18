package frc.robot.subsystems;
import frc.robot.Constants.SAMConstants;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Source Amp Mechanism that can intake game pieces from the source and eject to the Amp */
public class StrongArmMachine extends SubsystemBase {
    final TalonFX mIntakeMotor = new TalonFX(SAMConstants.kIntakeMoterID);
    final static ArmFeedforward mFeedForward = new ArmFeedforward(SAMConstants.kS, SAMConstants.kG, SAMConstants.kV, SAMConstants.kA);
    final DigitalInput mBreakbeam = new DigitalInput(9);
    
    final CANSparkMax mPivotMotor = new CANSparkMax(SAMConstants.kPivotMoterID, MotorType.kBrushless);
    final SparkAbsoluteEncoder mPivotEncoder;
    static SparkPIDController mPivotPID;

    static double mCurrentSetpoint = SAMPositions.HANDOFF_NOTE.value;
    boolean mIsBeamBroke = false;

    /** Enum for valid SAM pivot positions, measured in degrees */
    public enum SAMPositions {
        HANDOFF_NOTE(15.0),
        INTAKE_SOURCE(120.0),
        EJECT_AMP(100.0),
        ARB_VALUE(40.0);
        
        public final double value;
        private SAMPositions(double value) {
            this.value = value;
        }
    }

    public StrongArmMachine() {
        mPivotMotor.setIdleMode(IdleMode.kBrake);

        mPivotEncoder = mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mPivotEncoder.setPositionConversionFactor(SAMConstants.kPivotPositionConversionFactor);
        mPivotMotor.setInverted(true);
        mPivotPID = mPivotMotor.getPIDController();
        mPivotPID.setFeedbackDevice(mPivotEncoder);
        mPivotPID.setP(SAMConstants.kPivotP);
        mPivotPID.setI(SAMConstants.kPivotI);
        mPivotPID.setD(SAMConstants.kPivotD);
        mPivotPID.setFF(SAMConstants.kPivotFF);
        mPivotPID.setOutputRange(SAMConstants.kMinOutput, SAMConstants.kMaxOutput);

        mPivotMotor.burnFlash();

        mIntakeMotor.setInverted(true);
        
        SmartDashboard.putNumber("samP", SAMConstants.kPivotP);
        SmartDashboard.putNumber("samI", SAMConstants.kPivotI);
        SmartDashboard.putNumber("samD", SAMConstants.kPivotD);
        SmartDashboard.putNumber("samFF", SAMConstants.kPivotFF);
    }

    @Override
    public void periodic() {
        setShuffleboardPID();

        setPivotAngle(mCurrentSetpoint);

        SmartDashboard.putNumber("SAM Encoder", mPivotEncoder.getPosition());
        SmartDashboard.putNumber("SAM Setpoint", mCurrentSetpoint);
        SmartDashboard.putBoolean("Note In SAM", !mBreakbeam.get());
    }

    double gP  = SAMConstants.kPivotP,
           gI  = SAMConstants.kPivotI,
           gD  = SAMConstants.kPivotD,
           gFF = SAMConstants.kPivotFF;
    void setShuffleboardPID() {
        double p  = SmartDashboard.getNumber("samP", SAMConstants.kPivotP),
               i  = SmartDashboard.getNumber("samI", SAMConstants.kPivotI),
               d  = SmartDashboard.getNumber("samD", SAMConstants.kPivotD),
               ff = SmartDashboard.getNumber("samFF", SAMConstants.kPivotFF);
        if (Double.compare(p, gP) != 0) {
            mPivotPID.setP(p);
            gP = p;
        }

        if (Double.compare(i, gI) != 0) {
            mPivotPID.setI(i);
            gI = i;
        }

        if (Double.compare(d, gD) != 0) {
            mPivotPID.setD(d);
            gD = d;
        }

        if (Double.compare(ff, gFF) != 0) {
            mPivotPID.setFF(ff);
            gFF = ff;
        }
    }

    /**
     * Gets the current pivot setpoint
     * @return Setpoint, in degrees
     */
    public double getCurrentSetpoint() {
        return mCurrentSetpoint;
    }

    /**
     * Sets the current pivot setpoint
     * @param setpoint Setpoint angle, in degrees
     */
    public void setCurrentSetpoint(SAMPositions setpoint) {
        mCurrentSetpoint = setpoint.value;
    }

    /** Intended for debug/testing only */
    public Command movePivotManual(double axisOutput) {
        return run(() -> mPivotMotor.set(axisOutput));
    }

    /**
     * Sets the angle of the pivot via a control loop
     * @param degrees Angle of pivot, in degrees
     */
    private void setPivotAngle(double degrees) {
        double degreesClamped = MathUtil.clamp(degrees, SAMConstants.kLowerAngleLimit, SAMConstants.kUpperAngleLimit);

        mPivotPID.setReference(
            degreesClamped,
            ControlType.kPosition,
            0,
            -mFeedForward.calculate(Units.degreesToRadians(degreesClamped), 0.1)
        );
    }

    /**
     * Template command to run the SAM in various positions and speeds, with an endconditions
     * @param speed Speed to run SAM intake
     * @param position Position to set SAM pivot
     * @param endCondition Condition to end command
     * @return Command to run the SAM system
     * @see FunctionalCommand
     */
    private Command runSAM(double speed, SAMPositions position, BooleanSupplier endCondition) {
        return new FunctionalCommand(
            () -> setCurrentSetpoint(position), // init
            () -> { // exec
                // only run if reached setpoint
                if (Double.compare(mCurrentSetpoint, position.value) != 0) return;
                mIntakeMotor.set(speed);
            }, 
            isFinished -> { // end
                // default position
                setCurrentSetpoint(SAMPositions.HANDOFF_NOTE);
                mIntakeMotor.stopMotor();
            }, 
            endCondition, // isFinished 
            this
        );
    }

    public Command eject(double speed) {
        return new RunCommand(
            () -> mIntakeMotor.set(speed)
        );
    }

    /**
     * Checks if a game piece has fully passed by the break beam sensor
     * @return True if the gamepiece has passed the break beam
     */
    boolean hasNotePassedBreakbeam() {
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
    
    /**
     * Sets the SAM pivot position to HANDOFF_NOTE and 
     * runs the SAM intake inwards until a note has passed all the way through the breakbeam sensor
     * @return Command to set the pivot position and run the SAM intake
     * @see FunctionalCommand
     */
    public Command handoffFromIndexer() {
        return new SequentialCommandGroup(
            runSAM(SAMConstants.SAMspeedIn, SAMPositions.HANDOFF_NOTE, this::hasNotePassedBreakbeam),
            new ParallelRaceGroup(
                eject(-.5),
                new WaitCommand(.3)
            )
        )
    }

    /**
     * Sets the SAM pivot position to HANDOFF_NOTE and runs the SAM intake outwards.
     * @return Command to set the SAM pivot and run the SAM intake
     * @see FunctionalCommand
     */
    public Command handoffToIndexer() {
        return runSAM(SAMConstants.SAMspeedOut, SAMPositions.HANDOFF_NOTE, () -> false);
    }

    /**
     * Sets the SAM pivot position to INTAKE_SOURCE 
     * and runs the same intake inwards until the breakbeam sensor is hit
     * @return Command to set the SAM pivot and run the SAM intake
     * @see FunctionalCommand
     */
    public Command intakeSource() {
        return runSAM(SAMConstants.SAMspeedIn, SAMPositions.INTAKE_SOURCE, () -> mBreakbeam.get());
    }

    public Command prepSource() {
        return runSAM(0.0, SAMPositions.INTAKE_SOURCE, () -> false);
    }
    
    /**
     * Holds the SAM at the height of the amp, but does not eject the note
     * @return Command to set the SAM to the source position, but not run the SAM
     * @see FunctionalCommand
     */
    public Command holdAtAmp() {
        return runSAM(0, SAMPositions.EJECT_AMP, () -> false);
    }

    /**
     * While holding a note, dunk the note into the amp.
     * @return Command to dunk the note into the amp.
     * @see FunctionalCommand
     */
    public Command scoreAmp() {
        return runSAM(1.0, SAMPositions.EJECT_AMP, () -> false);
    }
}
