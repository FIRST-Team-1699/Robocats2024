package frc.team1699.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team1699.Constants.VisionConstants;
import frc.team1699.lib.vision.Limelight;

public class Vision {
    private static Vision vision;
    public static Vision getInstance() {
        if(vision == null) {
            vision = new Vision();
        }
        return vision;
    }

    private Limelight limelight;

    private Vision() {
        this.limelight = new Limelight(VisionConstants.kLimelightName, VisionConstants.kPipelineID, VisionConstants.kCameraPosition);
    }

    public boolean hasTargetInView() {
        return limelight.targetInView();
    }

    public Pose2d getPose2d() {
        return limelight.getPose2d();
    }

    public double getSpeakerX() {
        if(DriverStation.getAlliance().isPresent() && limelight.targetInView()) {
            Alliance alliance = DriverStation.getAlliance().get();
            if(alliance.equals(Alliance.Red)) {
                return getTagHorizontalOffset(4);
            } else {
                return getTagHorizontalOffset(8);
            }
        }
        return 0.0;
    }

    public double getSpeakerAngle() {
        if(DriverStation.getAlliance().isPresent() && limelight.targetInView()) {
            Alliance alliance = DriverStation.getAlliance().get();
            if(alliance.equals(Alliance.Red)) {
                limelight.setLimelightPipeline(8);
                
            } else {
                limelight.setLimelightPipeline(4);
            }
            return Math.atan(VisionConstants.kSpeakerAimHeight / limelight.getDistanceToTarget(VisionConstants.kSpeakerAimHeight));
        }
        return -1.0;
    }

    public double getTagHorizontalOffset(int tagID) {
        limelight.setLimelightPipeline(tagID);
        return limelight.getHorizontalOffset();
    }

    public double getTagVerticalOffset(int tagID) {
        limelight.setLimelightPipeline(tagID);
        return limelight.getVerticalOffset();
    }

    public void resetPipeline() {
        limelight.setLimelightPipeline(0);
    }
}
