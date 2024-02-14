package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterContants;

public class Shooter extends SubsystemBase {
    final CANSparkMax mLeftMotor = new CANSparkMax(ShooterContants.kLeftMotorID, MotorType.kBrushless);
    final CANSparkMax mRightMotor = new CANSparkMax(ShooterContants.kRightMotorID, MotorType.kBrushless);
    
    final CANSparkMax mPivotMotor = new CANSparkMax(ShooterContants.kPivotMotorID, MotorType.kBrushless);

    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    public Shooter() {
        mRightMotor.follow(mLeftMotor, true);
        
        mPivotMotor.setIdleMode(IdleMode.kBrake);

        mPivotEncoder = mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mPivotEncoder.setPositionConversionFactor(ShooterContants.kPivotPositionConversionFactor);
        
        mPivotPID = mPivotMotor.getPIDController();
        mPivotPID.setFeedbackDevice(mPivotEncoder);
        mPivotPID.setP(ShooterContants.kPivotP);
        mPivotPID.setI(ShooterContants.kPivotI);
        mPivotPID.setD(ShooterContants.kPivotD);
        mPivotPID.setFF(ShooterContants.kPivotFF);
        mPivotPID.setOutputRange(ShooterContants.kPivotMinOutput, ShooterContants.kPivotMaxOutput);
        
        mPivotMotor.burnFlash();
    }

    /**
     * Intended for testing/data collection use only.
     */
    public Command snapshotPosition() {
        return runOnce(() -> {
            SmartDashboard.putNumber("New Position", mPivotEncoder.getPosition());
        });
    }

    /**
     * Intended for testing/data collection use only.
     */
    public Command movePivotManual(double axisOutput) {
        return run(() -> {
            mPivotPID.setReference(axisOutput, ControlType.kPosition);
        });
    }

    /**
     * Sets the angle of the pivot.
     * @param degrees Desired angle setpoint, in degrees
     * @return runOnce command
     * @see InstantCommand
     */
    public Command setPivotAngle(double degrees) {
        // TODO: use our interpolating tree map instead
        return runOnce(() -> {
            mPivotPID.setReference(Units.degreesToRadians(degrees), ControlType.kPosition);
        });
    }

    /**
     * Sets both motors to a constant speed, intened to fire the gamepiece.
     * @return runEnd command
     * @see FunctionalCommand
     */
    public Command shootRing() {
        return runEnd(
            () -> {
                mLeftMotor.set(ShooterContants.kShooterSpeed);
            },
            () -> {
                mLeftMotor.stopMotor();
            }
        );
    }
}
