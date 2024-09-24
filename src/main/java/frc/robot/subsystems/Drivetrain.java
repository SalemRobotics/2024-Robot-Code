package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Direction;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
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

  final Pigeon2 mPigeon = new Pigeon2(DriveConstants.kPigeonID);

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

  boolean mIsFieldRelative, mIsRateLimited;

  /**
   * Constructs a drivetrain subsystem
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public Drivetrain(boolean fieldRelative, boolean rateLimit) {
    mIsFieldRelative = fieldRelative;
    mIsRateLimited = rateLimit;

    resetEncoders();
    mPigeon.reset();

    // configures holonomic drivetrain for auto 
    AutoBuilder.configureHolonomic(
      mOdometry::getPoseMeters, 
      this::resetOdometry, 
      this::getSpeeds, 
      this::setRobotRelativeStates, 
      DriveConstants.kPathConfig, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this
    );
  }

  public Command resetHeading() {
    return runOnce(() -> mPigeon.reset());
  }

  /**
   * Debug method to display driving and turning PID gains
   */
  void displayShuffleboardPID() {
    SmartDashboard.putNumber("Driving P", AutoConstants.kAutoDrivingP);
    SmartDashboard.putNumber("Driving I", AutoConstants.kAutoDrivingI);
    SmartDashboard.putNumber("Driving D", AutoConstants.kAutoDrivingD);
    SmartDashboard.putNumber("Driving IZone", AutoConstants.kAutoDrivingIZone);

    SmartDashboard.putNumber("Turning P", AutoConstants.kAutoTurningP);
    SmartDashboard.putNumber("Turning I", AutoConstants.kAutoTurningI);
    SmartDashboard.putNumber("Turning D", AutoConstants.kAutoTurningD);
    SmartDashboard.putNumber("Turning IZone", AutoConstants.kAutoTurningIZone);

    SmartDashboard.putNumber("HeadingP", DriveConstants.kHeadingP);
    SmartDashboard.putNumber("HeadingI", DriveConstants.kHeadingI);
    SmartDashboard.putNumber("HeadingD", DriveConstants.kHeadingD);
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

    SmartDashboard.putNumber("odometry X", mOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry Y", mOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("odometry rot", mOdometry.getPoseMeters().getRotation().getDegrees());
    RobotContainer.mField.setRobotPose(mOdometry.getPoseMeters());
  }

  /**
   * Gets swerve chassis speeds from swerve module states
   * @return {@link ChassisSpeeds}
   */
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      mFrontLeft.getState(),
      mFrontRight.getState(),
      mRearLeft.getState(),
      mRearRight.getState()
    );
  }

  /**
   * Sets swerve states relative to the robot from chassis speeds
   * @param robotRelativeSpeeds Chassis speeds relative to the robot
   * @see ChassisSpeeds
   */
  public void setRobotRelativeStates(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds));
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
   */
  public void drive(double xSpeed, double ySpeed, double rot) {

    double xSpeedCommanded;
    double ySpeedCommanded;
    if(DriverStation.isTestEnabled())
    {
      xSpeed /= 2;
      ySpeed /= 2;
    }

    if (mIsRateLimited) {
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
      } else if (angleDif > 0.85*Math.PI) {
        if (mCurrentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          mCurrentTranslationMag = mMagLimiter.calculate(0.0);
        } else {
          mCurrentTranslationDir = SwerveUtils.WrapAngle(mCurrentTranslationDir + Math.PI);
          mCurrentTranslationMag = mMagLimiter.calculate(inputTranslationMag);
        }
      } else {
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

    var swerveStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            mIsFieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, mPigeon.getRotation2d())
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    setModuleStates(swerveStates);
  }

  /** Get current translation magnitude */
  double getTranslationMag(double xSpeed, double ySpeed) {
    if (mIsRateLimited) {
      return mCurrentTranslationMag;
    }

    return Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
  }

  /** Get current tranlsation direction */
  double getTranslationDir(double xSpeed, double ySpeed) {
    if (mIsRateLimited) {
      return mCurrentTranslationDir;
    }

    return Math.atan2(ySpeed, xSpeed);
  }

  /**
   * Gets the difference in angle so that the setpoint is within 180 degrees of the gyro 
   * @param setpoint Angle setpoint, in degrees
   * @return Relative angle
   */
  double getPigeonModulus() {
    double angle = mPigeon.getRotation2d().getRadians();
    angle = MathUtil.angleModulus(angle);
    return Units.radiansToDegrees(angle);
  }

  /**
   * Command to track drivetrain rotation to an setpoint with a supplied measurement source
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param setpoint Setpoint to track drivetrain towards
   * @param measurement Measurement source of PIDCommand, a function returning a double
   * @return PIDCommand that tracks the drivetrain Yaw to a setpoint
   * @see DoubleSupplier
   * @see PIDCommand
   */
  Command trackSetpoint(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier setpoint, DoubleSupplier measurement) {
    return new PIDCommand(
      new PIDController(
        DriveConstants.kHeadingP,
        DriveConstants.kHeadingI,
        DriveConstants.kHeadingD
      ), 
      measurement, 
      setpoint,
      (receivedOutput) -> drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), receivedOutput),
      this
    );
  }

  double getSetpoint(DoubleSupplier distance) {
    if (Double.compare(distance.getAsDouble(), 0) == 0) {
      return 0.0;
    }
    
    return DriveConstants.kTrackingYawOffset;
  }

  /**
   * Tracks the drivetrain towards a vision target
   * @param yaw Supplied yaw of target
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @return Command to set the direction of the drivetrain
   * @see DoubleSupplier
   * @see PIDCommand
   */
  public Command trackTarget(DoubleSupplier distance, DoubleSupplier yaw, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    return trackSetpoint(xSpeed, ySpeed, () -> getSetpoint(distance), yaw);
  }

  /**
   * Tracks the drivetrain to a cardinal direction.
   * @param direction Cardinal direction to track
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @return Command to set the direction of the drivetrain
   * @see Direction
   * @see PIDCommand
   */
  public Command trackCardinal(Direction direction, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    return trackSetpoint(xSpeed, ySpeed, () -> direction.value, this::getPigeonModulus);
  }

  public Command chaseTarget(DoubleSupplier distance, DoubleSupplier yaw, BooleanSupplier isAtTarget) {
    return new SequentialCommandGroup(
      // make driving robot relative for this command
      new InstantCommand( () -> mIsFieldRelative = false ),
      new WaitUntilCommand(isAtTarget),
      // run track target with a fed X speed to chase a target, 
      // and end when target is aquired in indexer, or otherwise cancelled
      trackTarget(distance, yaw,
        () -> MathUtil.clamp(
          -distance.getAsDouble() * Math.cos( Units.degreesToRadians( yaw.getAsDouble() )),
          -1.0, 1.0),
        () -> 0.0
      ).until(isAtTarget),
      // set back to field relative
      new InstantCommand( () -> mIsFieldRelative = true )
    );
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   * @return Command to set the swerve modules
   * @see RunCommand
   */
  public Command setX() {
    return run(
      () -> setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        }));
  }

  /**
   * Sets the swerve ModuleStates.
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

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    mFrontLeft.resetEncoders();
    mRearLeft.resetEncoders();
    mFrontRight.resetEncoders();
    mRearRight.resetEncoders();
  }
}
