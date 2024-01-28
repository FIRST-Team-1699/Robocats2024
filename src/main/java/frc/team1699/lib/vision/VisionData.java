package frc.team1699.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionData {
    private final Double timestamp;
    private final Pose2d pose;

    public VisionData() {
        this.timestamp = -1.0;
        this.pose = new Pose2d();
    }

    public VisionData(Double timestamp, Pose2d pose) {
        this.timestamp = timestamp;
        this.pose = pose;
    }

    public Double getTimestamp() {
        return timestamp;
    }

    public Pose2d getPose2d() {
        return pose;
    }
}
