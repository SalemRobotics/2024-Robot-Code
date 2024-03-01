package frc.robot.subsystems;
import frc.robot.Constants.SAMConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StrongArmMachine extends SubsystemBase {
    final CANSparkMax mPivotMotor = new CANSparkMax(SAMConstants.SAMPivotMoterId, MotorType.kBrushless);
    final CANSparkMax mIntakeMotor = new CANSparkMax(SAMConstants.SAMIntakeMoterId, MotorType.kBrushless);
    final ArmFeedforward mFeedForward = new ArmFeedforward(SAMConstants.kS, SAMConstants.kG, SAMConstants.kV, SAMConstants.kA);
    
    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    enum SAMPositions {
        HANDOFF(0.0),
        INTAKE_SOURCE(0.0),
        EJECT_AMP(0.0);
        
        public final double value;
        private SAMPositions(double value) {
            this.value = value;    
        }
    }

    public StrongArmMachine() {
        mPivotMotor.restoreFactoryDefaults();

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
        
        SmartDashboard.putData(setShuffleboardPID());
    }

    Command setShuffleboardPID() {
        return runOnce(() -> {
            mPivotPID.setP(SmartDashboard.getNumber("samP", SAMConstants.kPivotP));
            mPivotPID.setI(SmartDashboard.getNumber("samI", SAMConstants.kPivotI));
            mPivotPID.setD(SmartDashboard.getNumber("samD", SAMConstants.kPivotD));
            mPivotPID.setFF(SmartDashboard.getNumber("samFF", SAMConstants.kPivotFF));
        });
    }

    /** Intended for debug/testing only */
    public Command snapshotPosition() {
        return runOnce(() -> {
            SmartDashboard.putNumber("New Position", mPivotEncoder.getPosition());
        });
    }

    /** Intended for debug/testing only */
    public Command movePivotManual(double axisOutput) {
        return run(() -> {
            mPivotMotor.set(axisOutput);
        });
    }

    public Command setPivotAngle(double degrees) {
        return runOnce(() -> {
            mPivotPID.setReference(
                mFeedForward.calculate(Units.degreesToRadians(degrees), 0), 
                ControlType.kPosition);
        });
    }

    public Command runIntake() {
        return runEnd(
            () -> {
                mIntakeMotor.set(SAMConstants.SAMIntakeSpeed);
            },
            () -> {
                mIntakeMotor.stopMotor();
            }
        );
    }

    public Command pivotUp() {
        return runEnd(
            () -> {
                mPivotMotor.set(-SAMConstants.SAMPivotSpeed);
            },
            () -> {
                mPivotMotor.stopMotor();
            }
        );
    }

    public Command pivotDown() {
        return runEnd(
            () -> {
                mPivotMotor.set(SAMConstants.SAMPivotSpeed);
            },
            () -> {
                mPivotMotor.stopMotor();
            }
        );
    }
    
    public Command runAmp() {
        return runEnd(
            () -> {
                mIntakeMotor.set(-SAMConstants.SAMIntakeSpeed);
            },
            () -> {
                mIntakeMotor.stopMotor();
            }
        );
    }
}
