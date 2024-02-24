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
        mFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile);
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
     * @see Optional
     */
    public Optional<Boolean> isTargetValid(PhotonTrackedTarget target) {
        Alliance allianceColor;
        try {
            allianceColor = DriverStation.getAlliance().orElseThrow();
        } catch (Exception e) {
            return null;
        }

        return Optional.of(
            VisionConstants.kValidFiducialIDs.get(allianceColor).contains(target.getFiducialId())
        );
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
            return null;
        }

        // copy and sort the fiducial ID list
        var fiducialIDsCopy = List.copyOf(target.fiducialIDsUsed);
        Collections.sort(fiducialIDsCopy);

        return Optional.of(
            fiducialIDsCopy.equals(VisionConstants.kValidFiducialIDs.get(allianceColor))
        );
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
            return null;
        }
        
        if (multiTag.estimatedPose.isPresent)
            return Optional.of(multiTag.estimatedPose.best);
        return null;
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
            return null;
        }

        return Optional.of(estimatedPose);
    }
    
    /**
     * Gets an estimated yaw from the robot pose to the Multitag targets
     * @return Yaw, if the target exists
     * @see Optional
     * @see Rotation2d
     */
    public Optional<Rotation2d> getMultitagYaw() {
        Transform3d targetTransform;
        try {
            targetTransform = getMultiTagTransform().orElseThrow();
        } catch (Exception e) {
            return null;
        }

        var targetPose = new Pose2d(
            targetTransform.getTranslation().toTranslation2d(),
            targetTransform.getRotation().toRotation2d()
        );

        Pose2d robotPose;
        try {
            robotPose = getMultitagRobotPose().orElseThrow().toPose2d();
        } catch (Exception e) {
            return null;
        }

        return Optional.of(PhotonUtils.getYawToPose(robotPose, targetPose));
    }

    /**
     * Gets an estimated distance from the robot pose to the Multitag target 
     * @return Distance, in meters, if the target exists
     * @see Optional
     */
    public Optional<Double> getMultitagDistance() {
        Transform3d targetTransform;
        try {
            targetTransform = getMultiTagTransform().orElseThrow();
        } catch (NoSuchElementException e) {
            System.err.println(e);
            return null;
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
            return null;
        }

        return Optional.of(PhotonUtils.getDistanceToPose(robotPose, targetPose));
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
        
        return Optional.of(currentResult.getBestTarget());
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
