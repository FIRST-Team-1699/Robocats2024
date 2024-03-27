package frc.team1699.lib.auto.modes;

import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team1699.lib.auto.events.Event;
import frc.team1699.lib.auto.events.FollowTrajectoryEvent;
import frc.team1699.lib.auto.events.RunIntakeEvent;
import frc.team1699.lib.auto.events.SpeakerShootSubEvent;
import frc.team1699.lib.auto.events.WaitUntilLoadedEvent;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;

public class ThreePieceClose extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public ThreePieceClose(Manipulator manipulator, Drive swerve) {
        PathPlannerTrajectory trajectoryOne = PathPlannerPath.fromPathFile("ThreePieceCloseOne").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryTwo = PathPlannerPath.fromPathFile("ThreePieceCloseTwo").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(45));
        PathPlannerTrajectory trajectoryThree = PathPlannerPath.fromPathFile("ThreePieceCloseThree").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryFour = PathPlannerPath.fromPathFile("ThreePieceCloseFour").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryFive = PathPlannerPath.fromPathFile("ThreePieceCloseFive").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        events = new ArrayList<Event>();
        events.add(new SpeakerShootSubEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryOne, swerve));
        events.add(new FollowTrajectoryEvent(trajectoryTwo, swerve));
        events.add(new WaitUntilLoadedEvent(manipulator));
        events.add(new SpeakerShootSubEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryThree, swerve));
        events.add(new FollowTrajectoryEvent(trajectoryFour, swerve));
        events.add(new WaitUntilLoadedEvent(manipulator));
        events.add(new SpeakerShootSubEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryFive, swerve));
        i = 0;

    }

    @Override
    public void initialize() {
        events.get(i).initialize();
    }

    @Override
    public void run() {
        if(i < events.size()) {
            Event currentEvent = events.get(i);
            if(currentEvent.isFinished()) {
                currentEvent.finish();
                i++;
                if(i < events.size()) {
                    events.get(i).initialize();
                }
            } else {
                currentEvent.update();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if(i >= events.size()) {
            return true;
        }
        return false;
    }

    @Override
    public void finish() {}
}
