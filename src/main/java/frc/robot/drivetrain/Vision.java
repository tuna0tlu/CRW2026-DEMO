package frc.robot.drivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

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

    public Optional<Pose2d> getEstimatedGlobalPose() {
        PhotonPipelineResult latestResult = camera.getLatestResult();
        if (!latestResult.hasTargets()) {
            return Optional.empty();
        }
        return poseEstimator.update(latestResult).map(est -> est.estimatedPose.toPose2d());
    }
}