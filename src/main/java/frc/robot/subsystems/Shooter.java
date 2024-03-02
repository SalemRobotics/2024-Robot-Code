package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
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

    final RelativeEncoder mleftEncoder;
    final RelativeEncoder mRightEncoder;

    final BangBangController mLeftController = new BangBangController(ShooterContants.kControllerErrorTolerance);
    final BangBangController mRightController = new BangBangController(ShooterContants.kControllerErrorTolerance);
    
    final CANSparkMax mPivotMotor = new CANSparkMax(ShooterContants.kPivotMotorID, MotorType.kBrushless);

    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    enum ShooterPositions {
        DEFAULT(0.0),
        SOURCE(0.0);

        public final double value;
        private ShooterPositions(double value) {
            this.value = value;
        }
    }

    public Shooter() {
        mRightMotor.restoreFactoryDefaults();
        mLeftMotor.restoreFactoryDefaults();
        mPivotMotor.restoreFactoryDefaults();

        mLeftMotor.setInverted(true);
        mRightMotor.setInverted(true);

        mleftEncoder = mLeftMotor.getEncoder();
        mRightEncoder = mRightMotor.getEncoder();

        mLeftMotor.burnFlash();
        mRightMotor.burnFlash();

        mLeftController.setSetpoint(ShooterContants.kLeftMotorSpeedSetpoint);
        mRightController.setSetpoint(ShooterContants.kRightMotorSpeedSetpoint);

        mPivotMotor.setIdleMode(IdleMode.kBrake);

        mPivotEncoder = mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        
        mPivotPID = mPivotMotor.getPIDController();
        mPivotPID.setFeedbackDevice(mPivotEncoder);
        mPivotPID.setP(ShooterContants.kPivotP);
        mPivotPID.setI(ShooterContants.kPivotI);
        mPivotPID.setD(ShooterContants.kPivotD);
        mPivotPID.setFF(ShooterContants.kPivotFF);
        mPivotPID.setOutputRange(ShooterContants.kPivotMinOutput, ShooterContants.kPivotMaxOutput);
        
        mPivotMotor.burnFlash();

        SmartDashboard.putNumber("shootP", ShooterContants.kPivotP);
        SmartDashboard.putNumber("shootI", ShooterContants.kPivotI);
        SmartDashboard.putNumber("shootD", ShooterContants.kPivotD);
        SmartDashboard.putNumber("shootFF", ShooterContants.kPivotFF);
    }

    @Override
    public void periodic() {
        setSmartDashboardPID();

        SmartDashboard.putNumber("leftVel", mleftEncoder.getVelocity());
        SmartDashboard.putNumber("rightVel", mRightEncoder.getVelocity());
    }

    double gP  = ShooterContants.kPivotP,
           gI  = ShooterContants.kPivotI,
           gD  = ShooterContants.kPivotD,
           gFF = ShooterContants.kPivotFF;
    void setSmartDashboardPID() {
        double p  = SmartDashboard.getNumber("shootP", gP),
               i  = SmartDashboard.getNumber("shootI", gI),
               d  = SmartDashboard.getNumber("shootD", gD),
               ff = SmartDashboard.getNumber("shootFF", gFF);

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
    public Command movePivotManual(DoubleSupplier axisOutput) {
        return run(() -> mPivotMotor.set(axisOutput.getAsDouble()));
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

        return runOnce(() -> 
            mPivotPID.setReference(
                ShooterContants.kPivotAngleMap.get(degreesClamped),
                ControlType.kPosition)
        );
    }

    public boolean atOutputThreshold() {
        return Double.compare(
            mleftEncoder.getVelocity(), 
            ShooterContants.kLeftMotorSpeedSetpoint * ShooterContants.kOutputVelocityThreshold) >= 0
        && Double.compare(
            mRightEncoder.getVelocity(), 
            ShooterContants.kRightMotorSpeedSetpoint * ShooterContants.kOutputVelocityThreshold) >= 0;
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
                    mLeftController.calculate(mleftEncoder.getVelocity())
                );

                mRightMotor.set(
                    mRightController.calculate(mleftEncoder.getVelocity())
                );
            },
            () -> {
                mLeftMotor.stopMotor();
                mRightMotor.stopMotor();
            }
        );
    }
}
