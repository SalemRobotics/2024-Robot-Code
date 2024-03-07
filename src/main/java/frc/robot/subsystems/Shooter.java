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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Shooter subsystem with two motors. Responsible for firing game pieces from the robot to the speaker goal.
 */
public class Shooter extends SubsystemBase {
    final CANSparkMax mLeftMotor = new CANSparkMax(ShooterConstants.kLeftMotorID, MotorType.kBrushless);
    final CANSparkMax mRightMotor = new CANSparkMax(ShooterConstants.kRightMotorID, MotorType.kBrushless);

    final RelativeEncoder mleftEncoder;
    final RelativeEncoder mRightEncoder;

    final BangBangController mLeftController = new BangBangController(ShooterConstants.kControllerErrorTolerance);
    final BangBangController mRightController = new BangBangController(ShooterConstants.kControllerErrorTolerance);
    
    final CANSparkMax mPivotMotor = new CANSparkMax(ShooterConstants.kPivotMotorID, MotorType.kBrushless);

    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    final ArmFeedforward mPivotFeedforward = new ArmFeedforward(
        1.0, ShooterConstants.kPivotKg, ShooterConstants.kPivotKv);

    enum ShooterPositions {
        DEFAULT(0.0),
        SOURCE(0.0);

        public final double value;
        private ShooterPositions(double value) {
            this.value = value;
        }
    }

    public Shooter() {
        mLeftMotor.setInverted(true);
        mRightMotor.setInverted(true);

        mleftEncoder = mLeftMotor.getEncoder();
        mRightEncoder = mRightMotor.getEncoder();

        mLeftMotor.burnFlash();
        mRightMotor.burnFlash();

        mLeftController.setSetpoint(ShooterConstants.kLeftMotorSpeedSetpoint);
        mRightController.setSetpoint(ShooterConstants.kRightMotorSpeedSetpoint);

        mPivotMotor.setIdleMode(IdleMode.kBrake);
        mPivotMotor.setInverted(true);

        mPivotEncoder = mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mPivotEncoder.setPositionConversionFactor(ShooterConstants.kPivotPositionConversionFactor);
        mPivotEncoder.setInverted(true);
        mPivotPID = mPivotMotor.getPIDController();
        mPivotPID.setFeedbackDevice(mPivotEncoder);
        mPivotPID.setP(ShooterConstants.kPivotP);
        mPivotPID.setI(ShooterConstants.kPivotI);
        mPivotPID.setD(ShooterConstants.kPivotD);
        mPivotPID.setOutputRange(ShooterConstants.kPivotMinOutput, ShooterConstants.kPivotMaxOutput);
        
        mPivotMotor.burnFlash();

        SmartDashboard.putNumber("shootP", ShooterConstants.kPivotP);
        SmartDashboard.putNumber("shootI", ShooterConstants.kPivotI);
        SmartDashboard.putNumber("shootD", ShooterConstants.kPivotD);
        SmartDashboard.putNumber("shootFF", ShooterConstants.kPivotFF);
    }

    @Override
    public void periodic() {
        setSmartDashboardPID();

        SmartDashboard.putNumber("Current Position", getFloorRelativeAngle());

        SmartDashboard.putNumber("leftVel", mleftEncoder.getVelocity());
        SmartDashboard.putNumber("rightVel", mRightEncoder.getVelocity());
    }

    double gP  = ShooterConstants.kPivotP,
           gI  = ShooterConstants.kPivotI,
           gD  = ShooterConstants.kPivotD,
           gFF = ShooterConstants.kPivotFF;
    void setSmartDashboardPID() {
        double p  = SmartDashboard.getNumber("shootP", gP),
               i  = SmartDashboard.getNumber("shootI", gI),
               d  = SmartDashboard.getNumber("shootD", gD),
               ff = SmartDashboard.getNumber("shootFF", gFF);

        if (Double.compare(p, gP) != 0) {
            mPivotPID.setP(p);
            SmartDashboard.putNumber("temp", mPivotPID.getP());
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
    public Command movePivotManual(DoubleSupplier axisOutput) {
        return run(
            () -> mPivotMotor.set(MathUtil.applyDeadband(axisOutput.getAsDouble(), 0.05))
        );
    }

    /**
     * Sets the angle of the pivot.
     * @param degrees Desired angle setpoint, in degrees
     * @return runOnce command
     * @see InstantCommand
     */
    public Command setPivotAngle(double degrees) {
        return run(() -> {
            // clamp input between lower and upper limits
            double degreesClamped = MathUtil.clamp(degrees, ShooterConstants.kLowerAngleLimit, ShooterConstants.kUpperAngleLimit);

            SmartDashboard.putNumber("Target Position", getEncoderRelativeAngle(degreesClamped));
            mPivotPID.setReference(
                getEncoderRelativeAngle(degreesClamped), 
                ControlType.kPosition, 
                0, 
                0
            );
        }
        );
    }

    public boolean atOutputThreshold() {
        return Double.compare(
            mleftEncoder.getVelocity(), 
            ShooterConstants.kLeftMotorSpeedSetpoint * ShooterConstants.kOutputVelocityThreshold) >= 0
        && Double.compare(
            mRightEncoder.getVelocity(), 
            ShooterConstants.kRightMotorSpeedSetpoint * ShooterConstants.kOutputVelocityThreshold) >= 0;
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

    private double getFloorRelativeAngle() {
        return mPivotEncoder.getPosition() - ShooterConstants.kEncoderOffset;
    }
    private double getEncoderRelativeAngle(double angle) {
        return ShooterConstants.kEncoderOffset + angle;
    }
}


