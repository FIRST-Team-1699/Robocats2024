package frc.team1699.lib.auto.modes;

import java.util.ArrayList;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team1699.lib.auto.events.DeadlineEvent;
import frc.team1699.lib.auto.events.Event;
import frc.team1699.lib.auto.events.FollowTrajectoryEvent;
import frc.team1699.lib.auto.events.RunIntakeEvent;
import frc.team1699.lib.auto.events.SpeakerAimLLEvent;
import frc.team1699.lib.auto.events.SpeakerShootLLEvent;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;

public class FivePieceBlue extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public FivePieceBlue(Manipulator manipulator, Drive swerve) {
        PathPlannerTrajectory trajectoryOne = PathPlannerPath.fromPathFile("BO5P1").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryTwo = PathPlannerPath.fromPathFile("BO5P2").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(40));
        PathPlannerTrajectory trajectoryThree = PathPlannerPath.fromPathFile("BO5P3").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryFour = PathPlannerPath.fromPathFile("BO5P4").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryFive = PathPlannerPath.fromPathFile("BO5P5").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(-38));
        events = new ArrayList<Event>();
        ArrayList<Event> arrOne = new ArrayList<Event>();
        arrOne.add(new FollowTrajectoryEvent(trajectoryOne, swerve));
        arrOne.add(new SpeakerAimLLEvent(manipulator));
        events.add(new DeadlineEvent(arrOne));
        events.add(new SpeakerShootLLEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryTwo, swerve));
        events.add(new SpeakerShootLLEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryThree, swerve));
        events.add(new SpeakerShootLLEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryFour, swerve));
        events.add(new SpeakerShootLLEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryFive, swerve));
        events.add(new SpeakerShootLLEvent(manipulator));
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
