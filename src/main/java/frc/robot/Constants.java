package frc.robot;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class ShooterConstants {
        public static final double kFreeSpinVelocity = 5880; // RPM

        public static final double kLeftMotorSpeedSetpoint = 0.9 * kFreeSpinVelocity; // RPM
        public static final double kRightMotorSpeedSetpoint = 0.7 * kFreeSpinVelocity; // RPM
        public static final double kControllerErrorTolerance = 0.1; // percent
        public static final double kOutputVelocityThreshold = 0.85; // percent

        public static final double kLeftMotorLobSpeed = 0.7; // percent
        public static final double kRightMotorLobSpeed = 0.5; // percent

        public static final int kPivotMotorID = 13;
        public static final int kRightMotorID = 14;
        public static final int kLeftMotorID = 15;

        public static final double kPivotP = 0.035;
        public static final double kPivotI = 0.0;
        public static final double kPivotD = 0.0;
        public static final double kPivotFF = 0.0;
        public static final double kPivotMaxOutput = 0.25;
        public static final double kPivotMinOutput = -0.25;

        public static final double kPivotPositionDegreesConversionFactor = 360.0;

        public static final double kUpperAngleLimitDegrees = 53.0;
        public static final double kLowerAngleLimitDegrees = 24.0;

        public static final double kEncoderOffsetDegrees = 231.9;

        public static final double kDefaultPivotDegrees = 53.0;

        /** Maps distances in meters to angles in degrees */
        public static final InterpolatingDoubleTreeMap kPivotDistanceAngleMap = new InterpolatingDoubleTreeMap() {{
            put(1.166, 53.0);
            put(1.273, 51.0);
            put(1.567, 47.5);
            put(2.028, 41.0);
            put(2.854, 35.0);
            put(3.3798, 30.0);
            put(4.280, 25.0);
            put(4.721, 24.0);
        }};
    }
    
    public static final class IndexerConstants {
        public static final int kIndexerLowerID = 11;
        public static final int kIndexerUpperID = 12;

        public static final int kBreakbeamID = 8;

        public static final double kIndexerSpeedIn = 1.0;
        public static final double kIndexerSpeedOut = -1.0;
    }
    
    public static final class IntakeConstants {
        public static final double kIntakeSpeedIn = 0.8;
        public static final double kIntakeSpeedOut = -0.5;

        public static final int kSparkMaxID = 10;
    }

    public static final class VisionConstants {
        /** Camera height in meters relative to where it is mounted on the robot */
        public static final double kCameraHeightMeters = Units.inchesToMeters(22);
        /**  Camera pitch in radians relative to where it is mounted on the robot */
        public static final double kCameraPitchRadians = Units.degreesToRadians(28);

        public static final double kCameraToRobotOffsetMeters = Units.inchesToMeters(11.75);

        public static final List<Integer> kValidFiducialIDs = List.of(4, 7);
        
        public static final double kCameraOffsetDegrees = 11.5;
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeedRadiansPerSec = 2 * Math.PI;

        public static final double kDirectionSlewRateRadiansPerSec = 1.2; 
        public static final double kMagnitudeSlewRatePercentPerSec = 2.5; 
        public static final double kRotationalSlewRatePercentPerSec = 2.0; 
        
        // Chassis configuration
        public static final boolean kGyroReversed = false;
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidthMeters = Units.inchesToMeters(23.5);
        // Distance between front and back wheels on robot
        public static final double kWheelBaseMeters = Units.inchesToMeters(23.5);
        // Radius of drive base. Equal to distance from center of robot to center of module.
        public static final double kDriveBaseRadiusMeters = Units.inchesToMeters(16.6);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2),
            new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2),
            new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2),
            new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2));

        public static final HolonomicPathFollowerConfig kPathConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(
                AutoConstants.kAutoDrivingP,
                AutoConstants.kAutoDrivingI,
                AutoConstants.kAutoDrivingD,
                AutoConstants.kAutoDrivingIZone
            ), 
            new PIDConstants(
                AutoConstants.kAutoTurningP,
                AutoConstants.kAutoTurningI,
                AutoConstants.kAutoTurningD,
                AutoConstants.kAutoTurningIZone
            ), 
            kMaxSpeedMetersPerSecond, 
            kDriveBaseRadiusMeters, 
            new ReplanningConfig(true, true)
        );

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        public static final int kPigeonID = 0;

        // misc PID controls
        public static final double kHeadingP = 0.0125;
        public static final double kHeadingI = 0.001;
        public static final double kHeadingD = 0.001;
        
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

        public static final double kDrivingP = 0.1;//0.04;
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

    public static final class AutoConstants{
        public static final double kAutoDrivingP = 5.5;
        public static final double kAutoDrivingI = 0;
        public static final double kAutoDrivingD = 0;
        public static final double kAutoDrivingIZone = 1000;
        
        //TODO: retune constants
        public static final double kAutoTurningP = 5;
        public static final double kAutoTurningI = 0;
        public static final double kAutoTurningD = 0;
        public static final double kAutoTurningIZone = 1000.0;

        /** Map of folder names to lists of auto command names */
        public static final HashMap<String, List<String>> kAutoFolders = new HashMap<>() {{
            put("Test Autos", List.of(
                "Test Auto",
                "Test Auto 2",       
                "Speed Test Auto",   
                "Test 2 piece auto",
                "Test Auto 3",       
                "Test Error Auto",   
                "Test Error Auto 2",
                "Test Error Auto 3",
                "Test Pickup Auto",  
                "Test Tumble Auto",  
                "Test Turn Auto" 
            ));
            put("Basic Autos", List.of(
                "Do Nothing",
                "Shoot + 1",            
                "Shoot + 2",            
                "Shoot + 3",            
                "Shoot and Do Nothing"
            ));
            put("Ampside Autos", List.of(
                "Amp 5 Note Auto",
                "Amp 6 Note Auto",
                "Amp 7 Note Auto"
            ));

            put("Non-Ampside Autos", List.of(
                "Non-Amp 5 Note",     
                "Non-Amp 6 Note Auto"
            ));

            put("Race Autos", List.of(
                "Amp Side Counterrace Auto",
                "Non-Amp Counterrace Auto",  
                "Amp Side Race Auto",        
                "Non-Amp Race Auto"
            ));

            put("Partner Autos", List.of(
                "Partner Auto"
            ));
        }};
    }

    public static final class SAMConstants {
        public static final double kA = 0.01;
        public static final double kS = 0.01;
        public static final double kG = 0.34;
        public static final double kV = 2.34;

        public static final double kSAMspeedIn = 0.5;
        public static final double kSAMspeedOut = -0.5;

        public static final int kPivotMoterID = 17;
        public static final int kIntakeMoterID = 16;

        public static final int kBreakbeamID = 9;

        public static final double kPivotSpeed = 0.5;

        public static final double kPivotPositionConversionFactor = 360.0;

        public static final double kPivotP = 0.015;
        public static final double kPivotI = 0;
        public static final double kPivotD = 0;
        public static final double kPivotFF = 0;
        public static final double kMaxOutput = 0.5;
        public static final double kMinOutput = -0.5;

        public static final double kUpperAngleLimit = 340.0;
        public static final  double kLowerAngleLimit = 208.0;

        public static final double kSetpointTolerance = 5.0;
    }
}
