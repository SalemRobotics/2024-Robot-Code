package frc.robot.subsystems;
import frc.robot.Constants.SAMConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StrongArmMachine extends SubsystemBase {
    final CANSparkMax mPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
    final CANSparkMax mIntakeMotor = new CANSparkMax(1, MotorType.kBrushless);
    final ArmFeedforward feedForward = new ArmFeedforward(SAMConstants.kS, SAMConstants.kG, SAMConstants.kV, SAMConstants.kA);

    public Command runIntake() {
        return runEnd(
            () -> {
                mIntakeMotor.set(SAMConstants.SAMspeed);
            },
            () -> {
                mIntakeMotor.stopMotor();
            }
        );
    }

    public Command pivotUp() {
        return runEnd(
            () -> {
                mPivotMotor.set(-SAMConstants.SAMspeed);
            },
            () -> {
                mPivotMotor.stopMotor();
            }
        );
    }

    public Command pivotDown() {
        return runEnd(
            () -> {
                mPivotMotor.set(SAMConstants.SAMspeed);
            },
            () -> {
                mPivotMotor.stopMotor();
            }
        );
    }
    
    public Command runAmp() {
        return runEnd(
            () -> {
                mIntakeMotor.set(-SAMConstants.SAMspeed);
            },
            () -> {
                mIntakeMotor.stopMotor();
            }
        );
    }
}
