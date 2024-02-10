package frc.team1699.lib.auto.modes;

import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import frc.team1699.lib.auto.events.Event;
import frc.team1699.lib.auto.events.FollowTrajectoryEvent;
import frc.team1699.lib.auto.events.RunIntakeEvent;
import frc.team1699.lib.auto.events.StopIntakeEvent;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Intake;

public class TestIntakeMode extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public TestIntakeMode(PathPlannerTrajectory trajectory, Intake intake, Drive swerve) {
        events.add(new RunIntakeEvent(intake));
        events.add(new FollowTrajectoryEvent(trajectory, swerve));
        events.add(new StopIntakeEvent(intake));

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
