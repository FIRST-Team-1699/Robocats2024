package frc.team1699.lib.auto.modes;

import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team1699.lib.auto.events.Event;
import frc.team1699.lib.auto.events.FollowTrajectoryEvent;
import frc.team1699.lib.auto.events.RunIntakeEvent;
import frc.team1699.lib.auto.events.SpeakerShootGoofyEvent;
import frc.team1699.lib.auto.events.SpeakerShootLLEvent;
import frc.team1699.lib.auto.events.SpeakerShootSubEvent;
import frc.team1699.lib.auto.events.StopIntakeEvent;
import frc.team1699.lib.auto.events.WaitUntilLoadedEvent;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;

public class RedOnePieceEscape extends AutoMode {
    private ArrayList<Event> events;
    private int i;
    private Manipulator manipulator;
    private Drive swerve;

    public RedOnePieceEscape(Manipulator manipulator, Drive swerve) {
        this.manipulator = manipulator;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        PathPlannerTrajectory trajectoryOne = PathPlannerPath.fromPathFile("RedOnePieceEscapeOne").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryTwo = PathPlannerPath.fromPathFile("RedOnePieceEscapeTwo").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(63.2));
        events = new ArrayList<Event>();
        events.add(new FollowTrajectoryEvent(trajectoryOne, swerve));
        events.add(new SpeakerShootGoofyEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryTwo, swerve));
        i = 0;
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