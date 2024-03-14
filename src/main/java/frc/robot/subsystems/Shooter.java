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
    //TODO: Set value
    public double mCurrentSetpoint = ShooterPositions.DEFAULT.value;
    
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
        mPivotEncoder.setPositionConversionFactor(ShooterConstants.kPivotPositionDegreesConversionFactor);
        mPivotEncoder.setInverted(true);
        mPivotPID = mPivotMotor.getPIDController();
        mPivotPID.setFeedbackDevice(mPivotEncoder);
        mPivotPID.setP(ShooterConstants.kPivotP);
        mPivotPID.setI(ShooterConstants.kPivotI);
        mPivotPID.setD(ShooterConstants.kPivotD);
        mPivotPID.setFF(ShooterConstants.kPivotFF);
        mPivotPID.setOutputRange(ShooterConstants.kPivotMinOutput, ShooterConstants.kPivotMaxOutput);
        
        mPivotMotor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current encoder angle", mPivotEncoder.getPosition());
        SmartDashboard.putNumber("Current floor angle", getFloorRelativeAngle());
        SmartDashboard.putNumber("Current setpoint", mCurrentSetpoint);
        SmartDashboard.putNumber("Encoder zero", mPivotEncoder.getZeroOffset());

        // continously set pivot reference
        setPivotAngle(mCurrentSetpoint);
    }

    public double getCurrentSetpoint() {
        return mCurrentSetpoint;
    }

    public void setCurrentSetpoint(double setpoint) {
        mCurrentSetpoint = setpoint;
    }

    /**
     * Sets the angle of the pivot.
     * @param degrees Desired angle setpoint, in degrees
     * @return runOnce command
     * @see InstantCommand
     */
    void setPivotAngle(double degrees) {
        // clamp input between lower and upper limits
        double degreesClamped = MathUtil.clamp(degrees, ShooterConstants.kLowerAngleLimitDegrees, ShooterConstants.kUpperAngleLimitDegrees);
        SmartDashboard.putNumber("degreesClamped", getEncoderRelativeAngle(degreesClamped));

        mPivotPID.setReference(
            getEncoderRelativeAngle(degreesClamped), 
            ControlType.kPosition
        );
    }

    /**
     * Checks if the shooter velocity is within a threshold of its max allowed speed
     * @return True if the shooter is at its output threshold
     */
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

    /**
     * Sets both motors to a constant speed, intened to fire the gamepiece.
     * @return runEnd command
     * @param targetDistance Distance of target to set the pivot angle to, should the target exist
     * @see FunctionalCommand
     */
    public Command shootRing(DoubleSupplier targetDistance) {
        return runEnd(
            () -> {
                double distance = targetDistance.getAsDouble();
                setCurrentSetpoint(
                    distance == 0 ? 
                    ShooterPositions.DEFAULT.value : 
                    ShooterConstants.kPivotDistanceAngleMap.get(targetDistance.getAsDouble())
                );

                mLeftMotor.set(
                    mLeftController.calculate(mleftEncoder.getVelocity())
                );

                mRightMotor.set(
                    mRightController.calculate(mleftEncoder.getVelocity())
                );
            },
            () -> {
                setCurrentSetpoint(ShooterPositions.DEFAULT.value);

                mLeftMotor.stopMotor();
                mRightMotor.stopMotor();
            }
        );
    }

    /**
     * Gets the floor relative angle by subtracting the encoder offset from the encoder position
     * @return the floor relative angle
     */
    private double getFloorRelativeAngle() {
        return mPivotEncoder.getPosition() - ShooterConstants.kEncoderOffsetDegrees;
    }

    /**
     * Gets the encoder relative angle by adding encoder offset to given angle
     * @param angle the desired angle
     * @return the encoder relative angle
     */
    private double getEncoderRelativeAngle(double angle) {
        return ShooterConstants.kEncoderOffsetDegrees + angle;
    }
}


