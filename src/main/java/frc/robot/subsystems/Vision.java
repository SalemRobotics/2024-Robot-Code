package frc.robot.subsystems;

import java.io.IOException;
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
import frc.robot.Constants.VisionConstants;

/**
 * Class containing a number of useful methods for working with PhotonLib. 
 */
public class Vision {
    final PhotonCamera mCamera = new PhotonCamera("placeholder");

    final PhotonPoseEstimator poseEstimator;

    final Pose2d mOdometryRobotPose;

    AprilTagFieldLayout mFieldLayout;

    public Vision(Pose2d robotPose) {
        mOdometryRobotPose = robotPose;
        
        try {
            mFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.err.println(e);
        }

        poseEstimator = new PhotonPoseEstimator(
            mFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            mCamera, 
            VisionConstants.kCameraPosition
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
        if (currentResult.hasTargets())
            return Optional.of(currentResult.getMultiTagResult());
        return null;
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
        } catch (NoSuchElementException e) {
            System.err.println(e);
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
        if (currentResult.hasTargets())
            return Optional.of(currentResult.getBestTarget());
        return null;
    }

    /**
     * Gets the distance from the robot to the target Multitag.
     * @return Distance, in meters, should the target exist. (I think)
     * @see Optional
     */
    public Optional<Double> getTargetDistance() {
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
            robotPose = getRobotPose().orElseThrow().toPose2d();
        } catch (Exception e) {
            System.err.println(e);
            return null;
        }

        return Optional.of(PhotonUtils.getDistanceToPose(robotPose, targetPose));
    }

    /**
     * Gets the yaw between the robot and target Multitag.
     * @return Yaw rotation, should it exist.
     * @see Rotation2d
     * @see Optional
     */
    public Optional<Rotation2d> getTargetYaw() {
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
            robotPose = getRobotPose().orElseThrow().toPose2d();
        } catch (NoSuchElementException e) {
            System.err.println(e);
            return null;
        }

        return Optional.of(PhotonUtils.getYawToPose(robotPose, targetPose));
    }

    /**
     * Gets the robot pose relative to tracked Multitag targets.
     * @return Robot pose.
     * @see Pose3d
     * @see Optional
     */
    public Optional<Pose3d> getRobotPose() {
        Pose3d estimatedPose;
        try {
            estimatedPose = poseEstimator.update().orElseThrow().estimatedPose;
        } catch (NoSuchElementException e) {
            System.err.println(e);
            return null;
        }

        return Optional.of(estimatedPose);
    }
}
