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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Shooter subsystem with two motors. Responsible for firing game pieces from the robot to the speaker goal.
 */
public class Shooter extends SubsystemBase {
    final CANSparkMax mLeftMotor = new CANSparkMax(ShooterConstants.kLeftMotorID, MotorType.kBrushless);
    final CANSparkMax mRightMotor = new CANSparkMax(ShooterConstants.kRightMotorID, MotorType.kBrushless);

    final RelativeEncoder mLeftEncoder;
    final RelativeEncoder mRightEncoder;

    final BangBangController mLeftController = new BangBangController(ShooterConstants.kControllerErrorTolerance);
    final BangBangController mRightController = new BangBangController(ShooterConstants.kControllerErrorTolerance);
    
    final CANSparkMax mPivotMotor = new CANSparkMax(ShooterConstants.kPivotMotorID, MotorType.kBrushless);

    final SparkAbsoluteEncoder mPivotEncoder;
    final SparkPIDController mPivotPID;

    double mCurrentSetpoint = ShooterConstants.kDefaultPivotDegrees;

    public Shooter() {
        mLeftMotor.setInverted(true);
        mRightMotor.setInverted(true);

        mLeftEncoder = mLeftMotor.getEncoder();
        mRightEncoder = mRightMotor.getEncoder();

        mLeftMotor.burnFlash();
        mRightMotor.burnFlash();

        mLeftController.setSetpoint(ShooterConstants.kLeftMotorSpeedSetpoint);
        mRightController.setSetpoint(ShooterConstants.kRightMotorSpeedSetpoint);

        mPivotMotor.setIdleMode(IdleMode.kBrake);
        mPivotMotor.setInverted(true);

        mPivotEncoder = mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mPivotEncoder.setPositionConversionFactor(ShooterConstants.kPivotPositionDegreesConversionFactor);
        mPivotEncoder.setZeroOffset(ShooterConstants.kPivotEncoderZero);
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
        // continously set pivot reference
        // mCurrentSetpoint = SmartDashboard.getNumber("Current setpoint", ShooterConstants.kDefaultPivotDegrees);
        setPivotAngle(mCurrentSetpoint);
    }

    /**
     * Prints various values regarding the shooter pivot to shuffleboard for debugging
     */
    void printDebug() {
        SmartDashboard.putNumber("Motor Output", mPivotMotor.getAppliedOutput());
        SmartDashboard.putNumber("Current setpoint", mCurrentSetpoint);
        SmartDashboard.putNumber("Current encoder angle", mPivotEncoder.getPosition());
        SmartDashboard.putNumber("Current floor angle", getFloorRelativeAngle());
        SmartDashboard.putNumber("Encoder zero", mPivotEncoder.getZeroOffset());
    }

    double gP  = ShooterConstants.kPivotP,
           gI  = ShooterConstants.kPivotI,
           gD  = ShooterConstants.kPivotD,
           gFF = ShooterConstants.kPivotFF;
    void setShuffleboardPID() {
        double p  = SmartDashboard.getNumber("Pivot P", gP),
               i  = SmartDashboard.getNumber("Pivot I", gI),
               d  = SmartDashboard.getNumber("Pivot D", gD),
               ff = SmartDashboard.getNumber("Pivot FF", gFF);
        if (Double.compare(p, gP) != 0) {
            gP = p;
            mPivotPID.setP(gP);
            SmartDashboard.putNumber("Pivot P", gP);
        }

        if (Double.compare(i, gI) != 0) {
            gI = i;
            mPivotPID.setI(gI);
            SmartDashboard.putNumber("Pivot I", gI);
        }

        if (Double.compare(d, gD) != 0) {
            gD = d;
            mPivotPID.setD(gD);
            SmartDashboard.putNumber("Pivot D", gD);
        }

        if (Double.compare(ff, gFF) != 0) {
            gFF = ff;
            mPivotPID.setFF(gFF);
            SmartDashboard.putNumber("Pivot FF", gFF);
        }
    }

    /**
     * Gets the current pivot setpoint value.
     * @return Pivot setpoint in degrees
     */
    public double getCurrentSetpoint() {
        return mCurrentSetpoint;
    }

    /**
     * Sets the current pivot setpoint value.
     * @param setpoint Angle in degrees to set the pivot setpoint to
     */
    public void setCurrentSetpoint(double setpoint) {
        mCurrentSetpoint = setpoint;
    }

    /**
     * Sets the angle of the pivot.
     * @param degrees Desired angle setpoint, in degrees
     */
    private void setPivotAngle(double degrees) {
        // clamp input between lower and upper limits
        double degreesClamped = MathUtil.clamp(degrees, 
        ShooterConstants.kLowerAngleLimitDegrees, ShooterConstants.kUpperAngleLimitDegrees);

        mPivotPID.setReference(
            getEncoderRelativeAngle(degreesClamped), 
            ControlType.kPosition
        );
    }

    /**
     * Returns a command that sets the shooter to the specified angle
     * @param degrees angle above the ground to set the shooter to
     * @return command that runs setShooterAngle to the angle in degrees
     */
    public Command setShooterAngle(double degrees) {
        return run(
            () -> {
                setPivotAngle(degrees);
            }
        );
    }

    /**
     * Checks if the shooter velocity is within a threshold of its max allowed speed
     * @return True if the shooter is at its output threshold
     */
    public boolean atOutputThreshold() {
        return Double.compare(
            mLeftEncoder.getVelocity(), 
            ShooterConstants.kLeftMotorSpeedSetpoint * ShooterConstants.kOutputVelocityThreshold) >= 0
        && Double.compare(
            mRightEncoder.getVelocity(), 
            ShooterConstants.kRightMotorSpeedSetpoint * ShooterConstants.kOutputVelocityThreshold) >= 0;
    }

    /**
     * Sets both motors to a constant speed, intended to fire the gamepiece.
     * @return runEnd command
     * @see FunctionalCommand
     */
    public Command shootRing() {
        return runEnd(
            () -> {
                mLeftMotor.set(
                    mLeftController.calculate(mLeftEncoder.getVelocity())
                );

                mRightMotor.set(
                    mRightController.calculate(mRightEncoder.getVelocity())
                );
            },
            () -> {
                mLeftMotor.stopMotor();
                mRightMotor.stopMotor();
            }
        );
    }

    /**
     * Lobs the game piece out of the robot at a lower speed, to act as a sort of "handoff" to other robots
     * @return runEnd command
     * @see FunctionalCommand
     */
    public Command lobRing() {
        return runEnd(
            () -> {
                setCurrentSetpoint(ShooterConstants.kLobSetpointDegrees);
                
                mLeftMotor.set(ShooterConstants.kLobSpeed);
                mRightMotor.set(ShooterConstants.kLobSpeed);
            }, 
            () -> {
                setCurrentSetpoint(ShooterConstants.kDefaultPivotDegrees);

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
                    ShooterConstants.kDefaultPivotDegrees : 
                    ShooterConstants.kPivotDistanceAngleMap.get(targetDistance.getAsDouble())
                );

                mLeftMotor.set(
                    mLeftController.calculate(mLeftEncoder.getVelocity())
                );

                mRightMotor.set(
                    mRightController.calculate(mRightEncoder.getVelocity())
                );
            },
            () -> {
                setCurrentSetpoint(ShooterConstants.kDefaultPivotDegrees);

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


