package frc.team1699.lib.auto.modes;

import java.util.ArrayList;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team1699.lib.auto.events.Event;
import frc.team1699.lib.auto.events.FollowTrajectoryEvent;
import frc.team1699.lib.auto.events.RunIntakeEvent;
import frc.team1699.lib.auto.events.StopIntakeEvent;
import frc.team1699.lib.auto.events.WaitEvent;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;

public class TestIntakeMode extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public TestIntakeMode(Manipulator manipulator, Drive swerve) {
        events = new ArrayList<Event>();
        events.add(new RunIntakeEvent(manipulator));
        PathPlannerTrajectory trajectoryOne = PathPlannerPath.fromPathFile("Auto Intake Test").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryTwo = PathPlannerPath.fromPathFile("TestIntakeSecond").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        events.add(new FollowTrajectoryEvent(trajectoryOne, swerve));
        events.add(new WaitEvent(0.5));
        events.add(new FollowTrajectoryEvent(trajectoryTwo, swerve));
        events.add(new WaitEvent(0.5));
        events.add(new StopIntakeEvent(manipulator));

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
