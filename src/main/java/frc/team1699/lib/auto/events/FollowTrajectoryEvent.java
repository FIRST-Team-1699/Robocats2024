package frc.team1699.lib.auto.events;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Drive.DriveState;

public class FollowTrajectoryEvent extends Event {
    private PathPlannerTrajectory trajectory;
    private Drive swerve;

    public FollowTrajectoryEvent(PathPlannerTrajectory trajectory, Drive swerve) {
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