package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionCamera;
import frc.robot.Constants.ShooterConstants;

public class Vision extends SubsystemBase {
    final VisionCamera mCamera;

    public Vision() {
        mCamera = new VisionCamera("Arducam_OV9281_USB_Camera");
    }

    @Override
    public void periodic() {
        printDebug();
    }

    /**
     * Sends various values from the camera to Shuffleboard for debugging.
     */
    private void printDebug() {
        SmartDashboard.putNumber("Target distance", getDistance());
        SmartDashboard.putNumber("Target Angle", ShooterConstants.kPivotDistanceAngleMap.get(getDistance()));
        
        try {
            SmartDashboard.putNumber("Target ID", mCamera.getBestTarget().get().getFiducialId());
            SmartDashboard.putString("Target Pose", mCamera.getTargetPose().get().toString());
        } catch (Exception e) {
            return;
        }
    }

    /**
     * Gets the target apriltag distance.
     * @return Distance, in meters, of the target. 0 if target is not found.
     */
    public double getDistance() {
        if (mCamera.getTargetDistance().isEmpty())
            return 0;

        return mCamera.getTargetDistance().orElseThrow();
    }
}
