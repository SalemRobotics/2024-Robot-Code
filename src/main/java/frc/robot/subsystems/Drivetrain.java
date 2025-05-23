package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.util.SwerveUtils;

/**
 * Swerve Drivetrain based off of REV Robotics MAXSwerve template. 
 */
public class Drivetrain extends SubsystemBase {
    // Create MAXSwerveModules
  final MAXSwerveModule mFrontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanID,
      DriveConstants.kFrontLeftTurningCanID,
      DriveConstants.kFrontLeftChassisAngularOffset);

  final MAXSwerveModule mFrontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanID,
      DriveConstants.kFrontRightTurningCanID,
      DriveConstants.kFrontRightChassisAngularOffset);

  final MAXSwerveModule mRearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanID,
      DriveConstants.kRearLeftTurningCanID,
      DriveConstants.kBackLeftChassisAngularOffset);

  final MAXSwerveModule mRearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanID,
      DriveConstants.kRearRightTurningCanID,
      DriveConstants.kBackRightChassisAngularOffset);

  final Pigeon2 mPigeon = new Pigeon2(0);

  // Slew rate filter variables for controlling lateral acceleration
  double mCurrentRotation = 0.0;
  double mCurrentTranslationDir = 0.0;
  double mCurrentTranslationMag = 0.0;

  SlewRateLimiter mMagLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRatePercentPerSec);
  SlewRateLimiter mRotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRatePercentPerSec);
  double mPrevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      mPigeon.getRotation2d(),
      new SwerveModulePosition[] {
          mFrontLeft.getPosition(),
          mFrontRight.getPosition(),
          mRearLeft.getPosition(),
          mRearRight.getPosition()
      });

  public Drivetrain() {
    resetEncoders();
    zeroHeading();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    mOdometry.update(
        mPigeon.getRotation2d(),
        new SwerveModulePosition[] {
            mFrontLeft.getPosition(),
            mFrontRight.getPosition(),
            mRearLeft.getPosition(),
            mRearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    mOdometry.resetPosition(
        mPigeon.getRotation2d(),
        new SwerveModulePosition[] {
            mFrontLeft.getPosition(),
            mFrontRight.getPosition(),
            mRearLeft.getPosition(),
            mRearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (mCurrentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRateRadiansPerSec / mCurrentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - mPrevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, mCurrentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        mCurrentTranslationDir = SwerveUtils.StepTowardsCircular(mCurrentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        mCurrentTranslationMag = mMagLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (mCurrentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          mCurrentTranslationMag = mMagLimiter.calculate(0.0);
        }
        else {
          mCurrentTranslationDir = SwerveUtils.WrapAngle(mCurrentTranslationDir + Math.PI);
          mCurrentTranslationMag = mMagLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        mCurrentTranslationDir = SwerveUtils.StepTowardsCircular(mCurrentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        mCurrentTranslationMag = mMagLimiter.calculate(0.0);
      }
      mPrevTime = currentTime;
      
      xSpeedCommanded = mCurrentTranslationMag * Math.cos(mCurrentTranslationDir);
      ySpeedCommanded = mCurrentTranslationMag * Math.sin(mCurrentTranslationDir);
      mCurrentRotation = mRotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      mCurrentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = mCurrentRotation * DriveConstants.kMaxAngularSpeedRadiansPerSec;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, mPigeon.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    mFrontLeft.setDesiredState(swerveModuleStates[0]);
    mFrontRight.setDesiredState(swerveModuleStates[1]);
    mRearLeft.setDesiredState(swerveModuleStates[2]);
    mRearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public Command setX() {
    return run(
      () -> {
        mFrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        mFrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        mRearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        mRearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      });
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    mFrontLeft.setDesiredState(desiredStates[0]);
    mFrontRight.setDesiredState(desiredStates[1]);
    mRearLeft.setDesiredState(desiredStates[2]);
    mRearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    mFrontLeft.resetEncoders();
    mRearLeft.resetEncoders();
    mFrontRight.resetEncoders();
    mRearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    mPigeon.setYaw(0.0);
  }
}
