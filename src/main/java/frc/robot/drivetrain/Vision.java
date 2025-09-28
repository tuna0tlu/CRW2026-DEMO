package frc.robot.drivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

public class Vision {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public Vision() {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.ROBOT_TO_CAM
        );
    }

    private Optional<PhotonPipelineResult> getLatestResultWithTargets() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }

        PhotonPipelineResult latestResult = results.get(results.size() - 1);
        if (!latestResult.hasTargets()) {
            return Optional.empty();
        }

        return Optional.of(latestResult);
    }

    public Optional<Pose2d> getEstimatedGlobalPose() {
        return getLatestResultWithTargets()
            .flatMap(poseEstimator::update)
            .map(est -> est.estimatedPose.toPose2d());
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return getLatestResultWithTargets().flatMap(poseEstimator::update);
    }
}