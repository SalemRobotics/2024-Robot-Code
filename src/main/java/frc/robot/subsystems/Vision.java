package frc.robot.subsystems;

import java.io.IOException;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    final PhotonCamera mCamera = new PhotonCamera("placeholder");

    final PhotonPoseEstimator poseEstimator;

    AprilTagFieldLayout mFieldLayout;

    // distance between calculated robot pose and target april tag pose
    public double targetDistance;

    // yaw between calculated robot pose and target april tag pose
    public Rotation2d targetYaw;

    public Vision() {
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

    public Optional<MultiTargetPNPResult> getMultiTagTarget() {
        var currentResult = mCamera.getLatestResult();
        if (currentResult.hasTargets())
            return Optional.of(currentResult.getMultiTagResult());
        return null;
    }

    public Optional<Transform3d> getMultiTagTransform() {
        var multiTag = getMultiTagTarget().get();
        if (multiTag.estimatedPose.isPresent)
            return Optional.of(multiTag.estimatedPose.best);
        return null;
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        var currentResult = mCamera.getLatestResult();
        if (currentResult.hasTargets())
            return Optional.of(currentResult.getBestTarget());
        return null;
    }

    public double getTargetDistance() {
        var targetTransform = getMultiTagTransform().get();

        var targetPose = new Pose2d(
            targetTransform.getTranslation().toTranslation2d(),
            targetTransform.getRotation().toRotation2d()
        );

        return PhotonUtils.getDistanceToPose(getRobotPose().toPose2d(), targetPose);
    }

    /**
     * Gets the field relative robot pose. 
     * TODO: try and implement a system to calculate error between odometry pose and apriltag pose and pick whichever has least error, or lerp between them
     * @return Pose3d representing robot pose 
     */
    public Pose3d getRobotPose() {
        return poseEstimator.update().get().estimatedPose;
    }
}
