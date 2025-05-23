package frc.robot;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;

/**
 * Class containing a number of useful methods for working with PhotonLib. 
 */
public class VisionCamera {
    final PhotonCamera mCamera;
    final AprilTagFieldLayout mFieldLayout;

    public VisionCamera(String cameraName) {
        mCamera = new PhotonCamera(cameraName);
        mFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    /**
     * Checks to see if an aquired single Apriltag target has a valid fiducial ID
     * @param target Valid single Apriltag target
     * @return True if target is valid (if it exists)
     * @see PhotonTrackedTarget
     */
    public boolean isTargetValid(PhotonTrackedTarget target) {
        if (DriverStation.getAlliance().isEmpty())
            return false;

        var allianceColor = DriverStation.getAlliance().orElseThrow();
        return VisionConstants.kValidFiducialIDs.get(allianceColor).contains(target.getFiducialId());
    }

    /**
     * Gets the best result of individual Apriltags.
     * @return The best Apriltag target, should it exist.
     * @see Optional
     * @see PhotonTrackedTarget
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        var currentResult = mCamera.getLatestResult();
        if (!currentResult.hasTargets())
            return Optional.empty(); 
        
        return Optional.of(currentResult.getBestTarget());
    }

    /**
     * Gets the target pose from the WPIlib AprilTag field layout based on 
     * the current target's fiducial ID
     * @return Target AprilTag pose
     * @see Pose3d
     * @see Optional
     */
    public Optional<Pose3d> getTargetPose() {
        if (getBestTarget().isEmpty())
            return Optional.empty();

        var target = getBestTarget().orElseThrow();
        return mFieldLayout.getTagPose(target.getFiducialId());
    }

    /**
     * Gets the distance from the robot to the target Apriltag.
     * @return Distance, in meters, should the target exist. (I think)
     * @see Optional
     */
    public Optional<Double> getTargetDistance() {
        var distance = getTargetDistance(VisionConstants.kCameraHeightMeters, VisionConstants.kCameraPitchRadians);
        if (distance.isEmpty())
            return Optional.empty();

        return distance;
    }

    /**
     * Gets the distance from the robot to the target Apriltag.
     * @param cameraHeight height of camera in meters
     * @param cameraPitch pitch of camera in radians
     * @return Distance, in meters, should the target exist. (I think)
     * @see Optional
     */
    public Optional<Double> getTargetDistance(double cameraHeight, double cameraPitch) {
        if (getTargetPitch().isEmpty() || getTargetPose().isEmpty())
            return Optional.empty();

        double targetPitch = getTargetPitch().orElseThrow();
        double targetHeight = getTargetPose().orElseThrow().getZ();

        return Optional.of(
            PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight, 
                targetHeight, 
                cameraPitch, 
                targetPitch));
    }

    /**
     * Gets the pitch between the camera and target Apriltag, in radians.
     * @return Pitch rotation in radians, should it exist.
     * @see Optional
     */
    public Optional<Double> getTargetPitch() {
        if (getBestTarget().isEmpty())
            return Optional.empty();

        double targetPitch = Units.degreesToRadians(
            getBestTarget().orElseThrow().getPitch());
            
        return Optional.of(targetPitch);
    }

    /**
     * Gets the yaw between the camera and target, in radians.
     * @return Yaw rotation in radians, should it exist.
     * @see Optional
     */
    public Optional<Double> getTargetYaw() {
        if (getBestTarget().isEmpty())
            return Optional.empty();

        double targetYaw = Units.degreesToRadians(
            getBestTarget().orElseThrow().getYaw());

        return Optional.of(targetYaw);
    }
}
