package frc.robot;

import java.util.Collections;
 import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.VisionConstants;

/**
 * Class containing a number of useful methods for working with PhotonLib. 
 */
public class VisionCamera {
    final PhotonCamera mCamera;

    public VisionCamera(String cameraName) {
        mCamera = new PhotonCamera(cameraName);
    }

    /**
     * Gets the Multitag target from the camera.
     * @return The Multitag target, should it exist.
     * @see Optional
     * @see MultiTargetPNPResult
     */
    public Optional<MultiTargetPNPResult> getMultiTagTarget() {
        var currentResult = mCamera.getLatestResult();
        if (!currentResult.hasTargets()) 
            return null;

        Alliance allianceColor;
        try {
            allianceColor = DriverStation.getAlliance().orElseThrow();
        } catch (Exception e) {
            return null;
        }

        var target = currentResult.getMultiTagResult();

        // copy and sort the fiducial ID list
        var fiducialIDsCopy = List.copyOf(target.fiducialIDsUsed);
        Collections.sort(fiducialIDsCopy);

        // if they don't match, discard the result
        if (!fiducialIDsCopy.equals(VisionConstants.kValidFiducialIDs.get(allianceColor))) 
            return null;

        return Optional.of(target);
    }

    /**
     * Gets the transform of the Multitag target.
     * @return transform of the Multitag target, should it exist.
     * @see Optional
     * @see Transform3d
     */
    public Optional<Transform3d> getMultiTagTransform() {
        MultiTargetPNPResult multiTag;
        try {
            multiTag = getMultiTagTarget().orElseThrow();
        } catch (Exception e) {
            return null;
        }
        
        if (multiTag.estimatedPose.isPresent)
            return Optional.of(multiTag.estimatedPose.best);
        return null;
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
            return null;

        Alliance allianceColor;
        try {
            allianceColor = DriverStation.getAlliance().orElseThrow();
        } catch (Exception e) {
            return null;
        }

        var target = currentResult.getBestTarget();
        // target fiducial ID does not match any of the valid IDs, discard result
        if (!VisionConstants.kValidFiducialIDs.get(allianceColor).contains(target.getFiducialId()))
            return null;
        
        return Optional.of(target);
    }

    /**
     * Gets the distance from the robot to the target Multitag.
     * @return Distance, in meters, should the target exist. (I think)
     * @see Optional
     */
    public Optional<Double> getTargetDistance() {
        double targetPitch;
        try {
            targetPitch = getTargetPitch().orElseThrow();
        } catch (Exception e) {
            return null;
        }

        return Optional.of(
            PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kCameraHeight, 
                Units.inchesToMeters(66), 
                VisionConstants.kCameraPitch, 
                targetPitch));
    }

    /**
     * Gets the pitch between the camera and target, in radians.
     * @return Pitch rotation, should it exist.
     * @see Optional
     */
    public Optional<Double> getTargetPitch() {
        double targetPitch;
        try {
            targetPitch = Units.degreesToRadians(getBestTarget().orElseThrow().getYaw());
        } catch (Exception e) {
            return null;
        }

        return Optional.of(targetPitch);
    }

    /**
     * Gets the yaw between the camera and target, in radians.
     * @return Yaw rotation, should it exist.
     * @see Optional
     */
    public Optional<Double> getTargetYaw() {
        double targetYaw;
        try {
            targetYaw = Units.degreesToRadians(getBestTarget().orElseThrow().getYaw());
        } catch (Exception e) {
            return null;
        }

        return Optional.of(targetYaw);
    }
}
