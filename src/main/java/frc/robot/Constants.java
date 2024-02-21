package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class ShooterContants {
        public static final double kShooterSpeed = 1.0;
        public static final double kAllowedOutputVelocity = 5800.0 * (85.0 / 100.0); // 85% of the measured peak RPM

        public static final int kLeftMotorID = 10;
        public static final int kRightMotorID = 13;
    }
    
    public static final class IndexerConstants {
        public static final double kIndexerSpeedIn = 1.0;
        public static final double kIndexerSpeedOut = -1.0;

        public static final int kIndexerIntakeID = 11;
        public static final int kIndexerShooterID = 12;
    }
    
    public static final class IntakeConstants {
        public static final double kIntakeSpeedIn = 1.0;
        public static final double kIntakeSpeedOut = -0.5;

        public static final int kSparkMaxID = 9;
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
        
        // Chassis configuration
        public static final boolean kGyroReversed = false;
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        // Distance between front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        // Radius of drive base. Equal to distance from center of robot to center of module.
        public static final double kDriveBaseRadius = Units.inchesToMeters(16.6);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

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
            kDriveBaseRadius, 
            new ReplanningConfig(true, true)
        );

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        public static final int kPigeonID = 0;

        // misc PID controls
        public static final double kHeadingP = 0.015;
        public static final double kHeadingI = 0.0;
        public static final double kHeadingD = 0.0;
        
        // SPARK MAX IDs
        public static final int kFrontLeftDrivingCanID = 7;
        public static final int kFrontRightDrivingCanID = 2;
        public static final int kRearLeftDrivingCanID = 6;
        public static final int kRearRightDrivingCanID = 3;

        public static final int kFrontLeftTurningCanID = 8;
        public static final int kFrontRightTurningCanID = 1;
        public static final int kRearLeftTurningCanID = 5;
        public static final int kRearRightTurningCanID = 4;
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
        public static final String kTestAuto = "Test Auto";
        public static final String kTestAuto2 = "Test Auto 2";
        
        public static final double kAutoDrivingP = 15;
        public static final double kAutoDrivingI = 1.5;
        public static final double kAutoDrivingD = 1;
        public static final double kAutoDrivingIZone = 1000;
        
        //oscillation in turning
        public static final double kAutoTurningP = 3.5;
        public static final double kAutoTurningI = 1;
        public static final double kAutoTurningD = 0.3;
        public static final double kAutoTurningIZone = 1000.0;

        // TODO: fill in auto names
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
}
