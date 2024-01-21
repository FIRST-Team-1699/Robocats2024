package frc.team1699.lib.vision;

import java.io.IOException;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionHandler {
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator poseEstimator;
    private PhotonCamera camera;

    public VisionHandler(String cameraName, Transform3d cameraPose, AprilTagFields tagField) {
        try {
            this.fieldLayout = AprilTagFieldLayout.loadFromResource(tagField.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            DriverStation.reportError("AprilTagFieldLayout failed to load", false);
        }
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, cameraPose);
    }

    public VisionData getEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedPoseOpt = poseEstimator.update();
        try {
            EstimatedRobotPose estimatedPose = estimatedPoseOpt.get();
            Pose2d estimatedPose2d = estimatedPose.estimatedPose.toPose2d();
            return new VisionData(estimatedPose.timestampSeconds, estimatedPose2d);
        } catch (NoSuchElementException e) {
            return new VisionData();
        }
    }
 }
