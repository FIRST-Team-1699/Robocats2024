package frc.team1699.lib.auto.modes;

import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team1699.lib.auto.HandledPath;
import frc.team1699.lib.auto.events.DeadlineEvent;
import frc.team1699.lib.auto.events.Event;
import frc.team1699.lib.auto.events.FollowTrajectoryEvent;
import frc.team1699.lib.auto.events.RunIntakeEvent;
import frc.team1699.lib.auto.events.SequentialEvent;
import frc.team1699.lib.auto.events.SpeakerAimLLEvent;
import frc.team1699.lib.auto.events.SpeakerShootLLEvent;
import frc.team1699.lib.auto.events.SpeakerShootSubEvent;
import frc.team1699.lib.auto.events.WaitUntilLoadedEvent;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;

public class OptimThreePiece extends AutoMode {
    private ArrayList<Event> events;
    private int i;
    private Manipulator manipulator;
    private Drive swerve;

    public OptimThreePiece(Manipulator manipulator, Drive swerve) {
        this.events = new ArrayList<Event>();
        this.i = 0;
        this.manipulator = manipulator;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        // HandledPath trajectoryOne = new HandledPath("O4P1");
        // HandledPath trajectoryTwo = new HandledPath("O4P2", Rotation2d.fromDegrees(40));
        // HandledPath trajectoryThree = new HandledPath("O4P3");
        
        PathPlannerTrajectory trajectoryOne = PathPlannerPath.fromPathFile("O4P1").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryTwo = PathPlannerPath.fromPathFile("O4P2").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(40));
        PathPlannerTrajectory trajectoryThree = PathPlannerPath.fromPathFile("O4P3").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryFour = PathPlannerPath.fromPathFile("O4P4").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        events.add(new SpeakerShootSubEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryOne, swerve));
        ArrayList<Event> arrayOne = new ArrayList<>();
        arrayOne.add(new WaitUntilLoadedEvent(manipulator));
        arrayOne.add(new SpeakerAimLLEvent(manipulator));
        SequentialEvent sequenceOne = new SequentialEvent(arrayOne);
        ArrayList<Event> arrayTwo = new ArrayList<>();
        arrayTwo.add(new FollowTrajectoryEvent(trajectoryTwo, swerve));
        arrayTwo.add(sequenceOne);
        events.add(new DeadlineEvent(arrayTwo));
        events.add(new SpeakerShootLLEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryThree, swerve));
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
