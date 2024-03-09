package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    public static final class ShooterConstants {
        public static final double kFreeSpinVelocity = 5880; // RPM

        public static final double kLeftMotorSpeedSetpoint = 0.9 * kFreeSpinVelocity; // RPM
        public static final double kRightMotorSpeedSetpoint = 0.7 * kFreeSpinVelocity; // RPM
        public static final double kControllerErrorTolerance = 0.1; // percent
        public static final double kOutputVelocityThreshold = 0.85; // percent

        public static final int kPivotMotorID = 13;
        public static final int kRightMotorID = 14;
        public static final int kLeftMotorID = 15;

        // TODO: tune constants
        public static final double kPivotP = 0.035;
        public static final double kPivotI = 0.0;
        public static final double kPivotD = 0.0;
        public static final double kPivotFF = 0.0;
        public static final double kPivotMaxOutput = 0.25;
        public static final double kPivotMinOutput = -0.25;

        public static final double kPivotPositionConversionFactor = 360.0; // Degrees

        public static final double kUpperAngleLimit = 53.0;
        public static final double kLowerAngleLimit = 25.0;

        public static final double kEncoderOffset = 231.9;

        public static final InterpolatingDoubleTreeMap kPivotDistanceAngleMap = new InterpolatingDoubleTreeMap() {{
            put(1.0, 48.0);
            put(2.0, 30.0);
        }};
    }
    
    public static final class IndexerConstants {
        public static final double kIndexerSpeed = 1.0;

        public static final int kIndexerLowerID = 11;
        public static final int kIndexerUpperID = 12;
        public static final double kIndexerSpeedIn = 1.0;
        public static final double kIndexerSpeedOut = -1.0;

        public static final int kIndexerIntakeID = 11;
        public static final int kIndexerShooterID = 12;
    }
    
    public static final class IntakeConstants {
        public static final double kIntakeSpeedIn = 0.25;
        public static final double kIntakeSpeedOut = -0.25;

        public static final int kSparkMaxID = 10;
    }

    public static final class VisionConstants {
        /** Camera height in meters relative to where it is mounted on the robot */
        public static final double kCameraHeight = Units.inchesToMeters(22); 
        /**  Camera pitch in radians relative to where it is mounted on the robot */
        public static final double kCameraPitch = Units.degreesToRadians(45);
        /** Height of the target in meters, in this case the Speaker */

        public static final Transform3d kCameraOffset = new Transform3d(
            0.0, 0.0, 0.0, 
            new Rotation3d(0.0, 0.0, 0.0)
        );

        public static final HashMap<Alliance, List<Integer>> kValidFiducialIDs = new HashMap<>() {{
            put(Alliance.Red, List.of(3, 4));
            put(Alliance.Blue, List.of(7, 8));
        }};
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 2.5; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
        
        public static final boolean kGyroReversed = false;
        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;
        
        // SPARK MAX IDs
        public static final int kFrontLeftDrivingCanID = 2;
        public static final int kFrontRightDrivingCanID = 3;
        public static final int kRearLeftDrivingCanID = 7;
        public static final int kRearRightDrivingCanID = 6;

        public static final int kFrontLeftTurningCanID = 1;
        public static final int kFrontRightTurningCanID = 4;
        public static final int kRearLeftTurningCanID = 8;
        public static final int kRearRightTurningCanID = 5;
    }

    public static final class SwerveConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
    }

    public static final class ControllerConstants {
        public static final int kDriverPort = 0;
        public static final double kDriveDeadband = 0.05;
        public static final int kOperatorPort = 1;
    }
}

