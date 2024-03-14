package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionCamera;
import frc.robot.Constants.ShooterConstants;

public class Vision extends SubsystemBase {
    VisionCamera mCamera;

    final Shooter mShooter;

    public Vision(Shooter shooter) {
        mShooter = shooter;

        try {
            mCamera = new VisionCamera("Arducam_OV9281_USB_Camera");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Target distance", getDistance());
        SmartDashboard.putNumber("Target Angle", ShooterConstants.kPivotDistanceAngleMap.get(getDistance()));
        
        mShooter.mCurrentSetpoint = ShooterConstants.kPivotDistanceAngleMap.get(getDistance());
        try {
            SmartDashboard.putNumber("Target ID", mCamera.getBestTarget().get().getFiducialId());
            SmartDashboard.putString("Target Pose", mCamera.getTargetPose().get().toString());
        } catch (Exception e) {
            return;
        }
    }

    /**
     * Gets the target apriltag distance.
     * @return in meters
     */
    public double getDistance() {
        try {
            return mCamera.getTargetDistance().orElseThrow();
        } catch (Exception e) {
            return 0;
        }
    }
}
