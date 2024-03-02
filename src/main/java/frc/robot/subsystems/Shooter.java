package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterContants;

/**
 * Shooter subsystem with two motors. Responsible for firing game pieces from the robot to the speaker goal.
 */
public class Shooter extends SubsystemBase {
    final CANSparkMax mLeftMotor = new CANSparkMax(ShooterContants.kLeftMotorID, MotorType.kBrushless);
    final CANSparkMax mRightMotor = new CANSparkMax(ShooterContants.kRightMotorID, MotorType.kBrushless);

    final BangBangController mLeftController = new BangBangController(ShooterContants.kControllerErrorTolerance);
    final BangBangController mRightController = new BangBangController(ShooterContants.kControllerErrorTolerance);
    
    final CANSparkMax mPivotMotor = new CANSparkMax(ShooterContants.kPivotMotorID, MotorType.kBrushless);

    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    public Shooter() {
        mRightMotor.restoreFactoryDefaults();
        mLeftMotor.restoreFactoryDefaults();
        mPivotMotor.restoreFactoryDefaults();

        mLeftMotor.setInverted(true);
        mRightMotor.setInverted(true);
        
        mLeftMotor.burnFlash();
        mRightMotor.burnFlash();

        mLeftController.setSetpoint(ShooterContants.kLeftMotorSpeedSetpoint);
        mRightController.setSetpoint(ShooterContants.kRightMotorSpeedSetpoint);

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

    @Override
    public void periodic() {
        setSmartDashboardPID();
    }

    void setSmartDashboardPID() {
        mPivotPID.setP(SmartDashboard.getNumber("shootP", ShooterContants.kPivotP));
        mPivotPID.setI(SmartDashboard.getNumber("shootI", ShooterContants.kPivotI));
        mPivotPID.setD(SmartDashboard.getNumber("shootD", ShooterContants.kPivotD));
        mPivotPID.setFF(SmartDashboard.getNumber("shootFF", ShooterContants.kPivotFF));
    }

    /**
     * Intended for testing/data collection use only.
     */
    public Command snapshotPosition() {
        return runOnce(() -> 
            SmartDashboard.putNumber("New Shoot Position", mPivotEncoder.getPosition())
        );
    }

    /**
     * Intended for testing/data collection use only.
     */
    public Command movePivotManual(double axisOutput) {
        return run(() -> mPivotMotor.set(axisOutput));
    }

    /**
     * Sets the angle of the pivot.
     * @param degrees Desired angle setpoint, in degrees
     * @return runOnce command
     * @see InstantCommand
     */
    public Command setPivotAngle(double degrees) {
        // clamp input between lower and upper limits
        double degreesClamped = MathUtil.clamp(degrees, ShooterContants.kLowerAngleLimit, ShooterContants.kUpperAngleLimit);

        // TODO: use our interpolating tree map instead (or not idk what to use it for just yet)
        return runOnce(() -> 
            mPivotPID.setReference(Units.degreesToRadians(degreesClamped), ControlType.kPosition)
        );
    }

    public boolean atOutputThreshold() {
        double leftOutput = mLeftMotor.getAppliedOutput() / ShooterContants.kLeftMotorSpeedSetpoint;
        double rightOutput = mRightMotor.getAppliedOutput() / ShooterContants.kRightMotorSpeedSetpoint;
        return Double.compare(leftOutput, ShooterContants.kOutputTolerance) >= 0
            && Double.compare(rightOutput, ShooterContants.kOutputTolerance) >= 0;
    }

    /**
     * Sets both motors to a constant speed, intened to fire the gamepiece.
     * @return runEnd command
     * @see FunctionalCommand
     */
    public Command shootRing() {
        return runEnd(
            () -> {
                mLeftMotor.set(
                    mLeftController.calculate(mLeftMotor.getAppliedOutput())
                );

                mRightMotor.set(
                    mRightController.calculate(mRightMotor.getAppliedOutput())
                );
            },
            () -> {
                mLeftMotor.stopMotor();
                mRightMotor.stopMotor();
            }
        );
    }
}
