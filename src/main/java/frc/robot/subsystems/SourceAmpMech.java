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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Source Amp Mechanism that can intake game pieces from the source and eject to the Amp */
public class SourceAmpMech extends SubsystemBase {
    final CANSparkMax mPivotMotor = new CANSparkMax(SAMConstants.kPivotMoterID, MotorType.kBrushless);
    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    double mCurrentSetpoint = SAMPositions.HANDOFF_NOTE.value;

    boolean mIsEnabled = false;

    /** Enum for valid SAM pivot positions, measured in degrees */
    public enum SAMPositions {
        HANDOFF_NOTE(214.0),
        INTAKE_SOURCE(290.0),
        EJECT_AMP(287.5);
        
        public final double value;
        private SAMPositions(double value) {
            this.value = value;
        }
    }

    public SourceAmpMech() {
        mPivotMotor.setIdleMode(IdleMode.kBrake);

        mPivotEncoder = mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mPivotEncoder.setPositionConversionFactor(SAMConstants.kPivotPositionConversionFactor);
        mPivotPID = mPivotMotor.getPIDController();
        mPivotMotor.setInverted(true);
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
        setPivotAngle(mCurrentSetpoint);
    }

    void printDebug() {
        SmartDashboard.putBoolean("SAM enabled", mIsEnabled);
        SmartDashboard.putBoolean("SAM Is At Setpoint", hasReachedSetpoint());
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

    public boolean hasReachedSetpoint() {
        return Math.abs(mPivotEncoder.getPosition() - mCurrentSetpoint) <= SAMConstants.kSetpointTolerance;
    }

    /**
     * Sets the angle of the pivot via a control loop
     * @param degrees Angle of pivot, in degrees
     */
    private void setPivotAngle(double degrees) {
        double degreesClamped = MathUtil.clamp(degrees, SAMConstants.kLowerAngleLimit, SAMConstants.kUpperAngleLimit);

        mPivotPID.setReference(
            degreesClamped,
            ControlType.kPosition
        );
    }

    public Command runSAM(SAMPositions position) {
        return runEnd(
            () -> {
                setEnabled(true);
                setCurrentSetpoint(position);
            },
            () -> setEnabled(true)
        );
    }

    public Command runSAM(SAMPositions position, BooleanSupplier endCondition) {
        return new FunctionalCommand(
            () -> {}, // init
            () -> {
                setEnabled(true);
                setCurrentSetpoint(position);
            }, // exec
            isFinished -> setEnabled(false), // end
            endCondition,
            this
        );
    }
}
