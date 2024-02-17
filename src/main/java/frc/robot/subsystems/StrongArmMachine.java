package frc.robot.subsystems;
import frc.robot.Constants.SAMConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StrongArmMachine extends SubsystemBase {
    final CANSparkMax mPivotMotor = new CANSparkMax(SAMConstants.SAMPivotMoterId, MotorType.kBrushless);
    final CANSparkMax mIntakeMotor = new CANSparkMax(SAMConstants.SAMIntakeMoterId, MotorType.kBrushless);
    final ArmFeedforward feedForward = new ArmFeedforward(SAMConstants.kS, SAMConstants.kG, SAMConstants.kV, SAMConstants.kA);
    
    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    public StrongArmMachine() {
        mPivotMotor.setIdleMode(IdleMode.kBrake);
        
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
