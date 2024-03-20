package frc.robot.subsystems;
import frc.robot.Constants.SAMConstants;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Source Amp Mechanism that can intake game pieces from the source and eject to the Amp */
public class SourceAmpMech extends SubsystemBase {
    final ArmFeedforward mFeedForward = new ArmFeedforward(SAMConstants.kS, SAMConstants.kG, SAMConstants.kV, SAMConstants.kA);
    
    final CANSparkMax mPivotMotor = new CANSparkMax(SAMConstants.kPivotMoterID, MotorType.kBrushless);
    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    double mCurrentSetpoint = SAMPositions.HANDOFF_NOTE.value;

    boolean mIsEnabled = false;

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

    public SourceAmpMech() {
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

    public boolean isEnabled() {
        return mIsEnabled;
    }

    public void setEnabled(boolean isEnabled) {
        mIsEnabled = isEnabled;
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

    public boolean hasReachedSetpoint() {
        return Double.compare(mPivotEncoder.getPosition(), mCurrentSetpoint) == 0;
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
            0.0// -mFeedForward.calculate(Units.degreesToRadians(degreesClamped), 0.1)
        );
    }

    public Command runSAM(SAMPositions position) {
        return runEnd(
            () -> {
                mIsEnabled = true;
                setCurrentSetpoint(position);
            },
            () -> mIsEnabled = false
        );
    }

    public Command runSAM(SAMPositions position, BooleanSupplier endCondition) {
        return new FunctionalCommand(
            () -> { // init
                mIsEnabled = true;
                setCurrentSetpoint(position);
            },
            () -> {}, // exec
            isFinished -> mIsEnabled = false,
            endCondition,
            this
        );
    }
}
