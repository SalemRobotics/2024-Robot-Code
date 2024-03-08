package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionCamera;

public class Vision extends SubsystemBase {
    VisionCamera mCamera;

    public Vision() {
        try {
            mCamera = new VisionCamera("USB-Camera");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target distance", getDistance());        
    }

    /**
     * Gets the target apriltag distance.
     * @return
     */
    public double getDistance() {
        try {
            return mCamera.getTargetDistance().orElseThrow();
        } catch (Exception e) {
            return 0;
        }
    }
}
