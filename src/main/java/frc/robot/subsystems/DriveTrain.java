package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
    CANSparkMax frontLeftSparkMax = new CANSparkMax(DriveTrainConstants.kFrontLeftSparkID, MotorType.kBrushless);
    CANSparkMax frontRightSparkMax = new CANSparkMax(DriveTrainConstants.kFrontRightSparkID, MotorType.kBrushless);
    CANSparkMax backLeftSparkMax = new CANSparkMax(DriveTrainConstants.kBackLeftSparkID, MotorType.kBrushless);
    CANSparkMax backRightSparkMax = new CANSparkMax(DriveTrainConstants.kBackRightSparkID, MotorType.kBrushless);

    DifferentialDrive drive = new DifferentialDrive(frontRightSparkMax, frontLeftSparkMax);
    
    public DriveTrain() {
        backLeftSparkMax.follow(backLeftSparkMax);
        frontRightSparkMax.setInverted(true);
        backRightSparkMax.follow(backRightSparkMax);
    }

    public Command arcadeDrive(double forward, double rotation) {
        return run(
            () -> {
                drive.arcadeDrive(forward, rotation);
            }
        );
    }
}