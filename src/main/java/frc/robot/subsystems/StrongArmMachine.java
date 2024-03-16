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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Source Amp Mechanism that can intake game pieces from the source and eject to the Amp */
public class StrongArmMachine extends SubsystemBase {
    final TalonFX mIntakeMotor = new TalonFX(SAMConstants.kIntakeMoterID);
    final ArmFeedforward mFeedForward = new ArmFeedforward(SAMConstants.kS, SAMConstants.kG, SAMConstants.kV, SAMConstants.kA);
    final DigitalInput mBreakbeam = new DigitalInput(SAMConstants.kBreakbeamID);
    
    final CANSparkMax mPivotMotor = new CANSparkMax(SAMConstants.kPivotMoterID, MotorType.kBrushless);
    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    double mCurrentSetpoint = SAMPositions.HANDOFF_NOTE.value;
    boolean mIsBeamBroke = false;

    /** Enum for valid SAM pivot positions, measured in degrees */
    public enum SAMPositions {
        HANDOFF_NOTE(15.0),
        INTAKE_SOURCE(120.0),
        EJECT_AMP(100.0);
        
        public final double value;
        private SAMPositions(double value) {
            this.value = value;
        }
    }

    public StrongArmMachine() {
        mPivotMotor.setIdleMode(IdleMode.kBrake);

        mPivotEncoder = mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mPivotEncoder.setPositionConversionFactor(SAMConstants.kPivotPositionConversionFactor);

        mPivotPID = mPivotMotor.getPIDController();
        mPivotPID.setFeedbackDevice(mPivotEncoder);
        mPivotPID.setP(SAMConstants.kPivotP);
        mPivotPID.setI(SAMConstants.kPivotI);
        mPivotPID.setD(SAMConstants.kPivotD);
        mPivotPID.setFF(SAMConstants.kPivotFF);
        mPivotPID.setOutputRange(SAMConstants.kMinOutput, SAMConstants.kMaxOutput);

        mPivotMotor.burnFlash();
        
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
            mFeedForward.calculate(Units.degreesToRadians(degreesClamped), 0)
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

    /**
     * Checks if a game piece has fully passed by the break beam sensor
     * @return True if the gamepiece has passed the break beam
     */
    boolean hasNotePassedBreakbeam() {
        if (mBreakbeam.get()) {
            mIsBeamBroke = true;
            return false;
        } 
        
        if (!mBreakbeam.get() && mIsBeamBroke) {
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
        return runSAM(SAMConstants.SAMspeedIn, SAMPositions.HANDOFF_NOTE, this::hasNotePassedBreakbeam);
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

    /**
     * Sets the SAM pivot position to EJECT_AMP and runs the SAM intake outwards
     * @return Command to set the SAM pivot and run the SAM intake
     * @see FunctionalCommand
     */
    public Command ejectAmp() {
        return runSAM(SAMConstants.SAMspeedOut, SAMPositions.EJECT_AMP, () -> false);
    }
}
