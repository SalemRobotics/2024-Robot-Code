package frc.robot;

import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    final AprilTagFieldLayout mFieldLayout;
    final PhotonPoseEstimator mPoseEstimator;

    public VisionCamera(String cameraName) throws IOException {
        mCamera = new PhotonCamera(cameraName);
        mFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        mPoseEstimator = new PhotonPoseEstimator(
            mFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            VisionConstants.kCameraOffset
        );
    }

    /**
     * Checks to see if an aquired single Apriltag target has a valid fiducial ID
     * @param target Valid single Apriltag target
     * @return True if target is valid (if it exists)
     * @see PhotonTrackedTarget
     */
    public boolean isTargetValid(PhotonTrackedTarget target) {
        Alliance allianceColor;
        try {
            allianceColor = DriverStation.getAlliance().orElseThrow();
        } catch (Exception e) {
            return false;
        }
            return VisionConstants.kValidFiducialIDs.get(allianceColor).contains(target.getFiducialId());
    }

    /**
     * Checks to see if an aquired Multitag target has valid fiducial IDs
     * @param target Valid Multitag target
     * @return True if target is valid (if it exists)
     * @see MultiTargetPNPResult
     * @see Optional
     */
    public Optional<Boolean> isTargetValid(MultiTargetPNPResult target) {
        Alliance allianceColor;
        try {
            allianceColor = DriverStation.getAlliance().orElseThrow();
        } catch (Exception e) {
            return Optional.empty();
        }

        // copy and sort the fiducial ID list
        var fiducialIDsCopy = List.copyOf(target.fiducialIDsUsed);
        Collections.sort(fiducialIDsCopy);

        return Optional.of(
            fiducialIDsCopy.equals(VisionConstants.kValidFiducialIDs.get(allianceColor))
        );
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

    public Optional<Pose3d> getTargetPose() {
        PhotonTrackedTarget target;
        try {
            target = getBestTarget().orElseThrow();
        } catch (Exception e) {
            return null;
        }

        return mFieldLayout.getTagPose(target.getFiducialId());
    }

    /**
     * Gets the distance from the robot to the target Apriltag.
     * @return Distance, in meters, should the target exist. (I think)
     * @see Optional
     */
    public Optional<Double> getTargetDistance() {
        double targetPitch, targetHeight;
        try {
            targetPitch = getTargetPitch().orElseThrow();
            targetHeight = getTargetPose().orElseThrow().getZ();
        } catch (Exception e) {
            return Optional.empty();
        }

        return Optional.of(
            PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kCameraHeight, 
                targetHeight, 
                VisionConstants.kCameraPitch, 
                targetPitch));
    }

    /**
     * Gets the distance from the robot to the target Apriltag.
     * @param cameraHeight height of camera in meters
     * @param cameraPitch pitch of camera in radians
     * @return Distance, in meters, should the target exist. (I think)
     * @see Optional
     */
    public Optional<Double> getTargetDistance(double cameraHeight, double cameraPitch) {
        double targetPitch, targetHeight;
        try {
            targetPitch = getTargetPitch().orElseThrow();
            targetHeight = getTargetPose().orElseThrow().getZ();
        } catch (Exception e) {
            return null;
        }

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
        double targetPitch;
        try {
            targetPitch = Units.degreesToRadians(getBestTarget().orElseThrow().getPitch());
        } catch (Exception e) {
            return Optional.empty();
        }

        return Optional.of(targetPitch);
    }

    /**
     * Gets the pitch from the camera to the top of the speaker from the apriltag
     * @return Pitch rotation in radians, should it exist
     * @see Optional
     */
    public Optional<Double> getSpeakerPitch() {
        double targetDistance;
        try {
            targetDistance = getTargetDistance().orElseThrow();
        } catch (Exception e) {
            return Optional.empty();
        }

        return Optional.of(
            Math.atan2(VisionConstants.kTargetHeight, targetDistance)
        );
    }

    /**
     * Gets the yaw between the camera and target, in radians.
     * @return Yaw rotation in radians, should it exist.
     * @see Optional
     */
    public Optional<Double> getTargetYaw() {
        double targetYaw;
        try {
            targetYaw = Units.degreesToRadians(getBestTarget().orElseThrow().getYaw());
        } catch (Exception e) {
            return Optional.empty();
        }

        return Optional.of(targetYaw);
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
            return Optional.empty();

        return Optional.of(currentResult.getMultiTagResult());
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
            return Optional.empty();
        }
        
        if (multiTag.estimatedPose.isPresent)
            return Optional.of(multiTag.estimatedPose.best);
        return Optional.empty();
    }

    /**
     * Gets an estimated robot pose from tracked Multitag targets
     * @return Robot pose, if the target exists
     * @see Optional
     * @see Pose3d
     */
    public Optional<Pose3d> getMultitagRobotPose() {
        Pose3d estimatedPose;
        try {
            estimatedPose = mPoseEstimator.update().orElseThrow().estimatedPose;
        } catch (Exception e) {
            return Optional.empty();
        }

        return Optional.of(estimatedPose);
    }
    
    /**
     * Gets an estimated yaw from the robot pose to the Multitag targets
     * @return Yaw, if the target exists
     * @see Optional
     * @see Rotation2d
     */
    public Optional<Rotation2d> getMultitagRelativeYaw() {
        Transform3d targetTransform;
        try {
            targetTransform = getMultiTagTransform().orElseThrow();
        } catch (Exception e) {
            return Optional.empty();
        }

        var targetPose = new Pose2d(
            targetTransform.getTranslation().toTranslation2d(),
            targetTransform.getRotation().toRotation2d()
        );

        Pose2d robotPose;
        try {
            robotPose = getMultitagRobotPose().orElseThrow().toPose2d();
        } catch (Exception e) {
            return Optional.empty();
        }

        return Optional.of(PhotonUtils.getYawToPose(robotPose, targetPose));
    }

    /**
     * Gets an estimated distance from the robot pose to the Multitag target 
     * @return Distance, in meters, if the target exists
     * @see Optional
     */
    public Optional<Double> getMultitagRelativeDistance() {
        Transform3d targetTransform;
        try {
            targetTransform = getMultiTagTransform().orElseThrow();
        } catch (NoSuchElementException e) {
            System.err.println(e);
            return Optional.empty();
        }

        var targetPose = new Pose2d(
            targetTransform.getTranslation().toTranslation2d(),
            targetTransform.getRotation().toRotation2d()
        );

        Pose2d robotPose;
        try {
            robotPose = getMultitagRobotPose().orElseThrow().toPose2d();
        } catch (Exception e) {
            System.err.println(e);
            return Optional.empty();
        }

        return Optional.of(PhotonUtils.getDistanceToPose(robotPose, targetPose));
    }

}
