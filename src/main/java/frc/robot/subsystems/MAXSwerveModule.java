// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.Constants.SwerveConstants;

public class MAXSwerveModule {
  final CANSparkMax mDrivingSparkMax;
  final CANSparkMax mTurningSparkMax;

  final RelativeEncoder mDrivingEncoder;
  final AbsoluteEncoder mTurningEncoder;

  final SparkPIDController mDrivingPIDController;
  final SparkPIDController mTurningPIDController;

  double mChassisAngularOffset = 0;
  SwerveModuleState mDesiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    mDrivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    mTurningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // mDrivingSparkMax.restoreFactoryDefaults();
    // mTurningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    mDrivingEncoder = mDrivingSparkMax.getEncoder();
    mTurningEncoder = mTurningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    mDrivingPIDController = mDrivingSparkMax.getPIDController();
    mTurningPIDController = mTurningSparkMax.getPIDController();
    mDrivingPIDController.setFeedbackDevice(mDrivingEncoder);
    mTurningPIDController.setFeedbackDevice(mTurningEncoder);

    // PID constant defaults

    mDrivingPIDController.setP(SwerveConstants.kDrivingP);
    mDrivingPIDController.setI(SwerveConstants.kDrivingI);
    mDrivingPIDController.setD(SwerveConstants.kDrivingD);
    mDrivingPIDController.setFF(SwerveConstants.kDrivingFF);
    mDrivingPIDController.setOutputRange(SwerveConstants.kDrivingMinOutput, SwerveConstants.kDrivingMaxOutput);
    
    mTurningPIDController.setP(SwerveConstants.kTurningP);
    mTurningPIDController.setI(SwerveConstants.kTurningI);
    mTurningPIDController.setD(SwerveConstants.kTurningD);
    mTurningPIDController.setFF(SwerveConstants.kTurningFF);
    mTurningPIDController.setOutputRange(SwerveConstants.kTurningMinOutput, SwerveConstants.kTurningMaxOutput);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    mDrivingEncoder.setPositionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor);
    mDrivingEncoder.setVelocityConversionFactor(SwerveConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    mTurningEncoder.setPositionConversionFactor(SwerveConstants.kTurningEncoderPositionFactor);
    mTurningEncoder.setVelocityConversionFactor(SwerveConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    mTurningEncoder.setInverted(SwerveConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    mTurningPIDController.setPositionPIDWrappingEnabled(true);
    mTurningPIDController.setPositionPIDWrappingMinInput(SwerveConstants.kTurningEncoderPositionPIDMinInput);
    mTurningPIDController.setPositionPIDWrappingMaxInput(SwerveConstants.kTurningEncoderPositionPIDMaxInput);

    mDrivingSparkMax.setIdleMode(SwerveConstants.kDrivingMotorIdleMode);
    mTurningSparkMax.setIdleMode(SwerveConstants.kTurningMotorIdleMode);
    mDrivingSparkMax.setSmartCurrentLimit(SwerveConstants.kDrivingMotorCurrentLimit);
    mTurningSparkMax.setSmartCurrentLimit(SwerveConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    mDrivingSparkMax.burnFlash();
    mTurningSparkMax.burnFlash();

    mChassisAngularOffset = chassisAngularOffset;
    mDesiredState.angle = new Rotation2d(mTurningEncoder.getPosition());
    mDrivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(mDrivingEncoder.getVelocity(),
        new Rotation2d(mTurningEncoder.getPosition() - mChassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        mDrivingEncoder.getPosition(),
        new Rotation2d(mTurningEncoder.getPosition() - mChassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(mChassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(mTurningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    mDrivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    mTurningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    mDesiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    mDrivingEncoder.setPosition(0);
  }
}