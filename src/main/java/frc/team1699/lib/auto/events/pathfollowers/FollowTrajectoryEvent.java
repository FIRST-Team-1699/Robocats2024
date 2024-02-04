package frc.team1699.lib.auto.events.pathfollowers;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team1699.lib.auto.events.Event;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Drive.DriveState;

public class FollowTrajectoryEvent extends Event {
    private PathPlannerTrajectory trajectory;
    private Drive swerve;

    public static FollowTrajectoryEvent getSWFToCenterPreset(Drive swerve) {
        return new FollowTrajectoryEvent(loadTrajectory("SWFToMiddlePreset"), swerve);
    }

    private static PathPlannerTrajectory loadTrajectory(String trajName) {
        return PathPlannerPath.fromPathFile(trajName).getTrajectory(new ChassisSpeeds(), new Rotation2d());
    }

    private FollowTrajectoryEvent(PathPlannerTrajectory trajectory, Drive swerve) {
        this.trajectory = trajectory;
        this.swerve = swerve;
    }

    public void initialize() {
        swerve.setTrajectory(trajectory);
        swerve.setWantedState(DriveState.FOLLOW_TRAJ);
    }

    public void update() {}
    
    public boolean isFinished() {
        if(swerve.doneWithTraj() == true) {
            return true;
        }
        return false;
    }

    public void finish() {
        swerve.setWantedState(DriveState.LOCK);
    }
}