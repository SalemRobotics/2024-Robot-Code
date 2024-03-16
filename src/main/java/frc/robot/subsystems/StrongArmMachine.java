package frc.robot.subsystems;
import frc.robot.Constants.SAMConstants;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StrongArmMachine extends SubsystemBase {
    final CANSparkMax mPivotMotor = new CANSparkMax(SAMConstants.kPivotMoterID, MotorType.kBrushless);
    final CANSparkMax mIntakeMotor = new CANSparkMax(SAMConstants.kIntakeMoterID, MotorType.kBrushless);
    final ArmFeedforward mFeedForward = new ArmFeedforward(SAMConstants.kS, SAMConstants.kG, SAMConstants.kV, SAMConstants.kA);
    
    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    final DigitalInput mBreakbeam = new DigitalInput(SAMConstants.kBreakbeamID);

    double mCurrentSetpoint;

    //TODO: when intaking from source, run SAM until breakbeam is triggered and then stop the SAM
    //TODO: when handing off from indexer to SAM, run SAM until breakbeam is triggered and stop running when breakbeam is exited

    enum SAMPositions {
        HANDOFF_NOTE(0.0),
        INTAKE_SOURCE(0.0),
        EJECT_AMP(0.0);
        
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

    public double getCurrentSetpoint() {
        return mCurrentSetpoint;
    }

    public void setCurrentSetpoint(double setpoint) {
        mCurrentSetpoint = setpoint;
    }

    /** Intended for debug/testing only */
    public Command movePivotManual(double axisOutput) {
        return run(() -> mPivotMotor.set(axisOutput));
    }

    public void setPivotAngle(double degrees) {
        double degreesClamped = MathUtil.clamp(degrees, SAMConstants.kLowerAngleLimit, SAMConstants.kUpperAngleLimit);

        mPivotPID.setReference(
            degreesClamped,
            ControlType.kPosition,
            0,
            mFeedForward.calculate(Units.degreesToRadians(degreesClamped), 0)
        );
    }

    public Command runIntake(double speed) {
        return runEnd(
            () -> {
                mIntakeMotor.set(speed);
            },
            () -> {
                mIntakeMotor.stopMotor();
            }
        );
    }
}
