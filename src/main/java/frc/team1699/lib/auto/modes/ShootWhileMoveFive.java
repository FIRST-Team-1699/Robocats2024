package frc.team1699.lib.auto.modes;

import java.util.ArrayList;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team1699.lib.auto.events.Event;
import frc.team1699.lib.auto.events.FollowTrajectoryEvent;
import frc.team1699.lib.auto.events.ParallelEvent;
import frc.team1699.lib.auto.events.RunIntakeEvent;
import frc.team1699.lib.auto.events.SequentialEvent;
import frc.team1699.lib.auto.events.SpeakerShootLLEvent;
import frc.team1699.lib.auto.events.WaitUntilLoadedEvent;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;

public class ShootWhileMoveFive extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public ShootWhileMoveFive(Manipulator manipulator, Drive swerve) {
        PathPlannerTrajectory trajectoryOne = PathPlannerPath.fromPathFile("FivePieceOnePlusTwo").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryTwo = PathPlannerPath.fromPathFile("FivePieceThreePlusFour").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryThree = PathPlannerPath.fromPathFile("RO4P5").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(30));
        PathPlannerTrajectory trajectoryFour = PathPlannerPath.fromPathFile("B5P5").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(10));
        events = new ArrayList<Event>();
        events.add(new RunIntakeEvent(manipulator));
        ArrayList<Event> arrayOne = new ArrayList<>();
        arrayOne.add(new FollowTrajectoryEvent(trajectoryOne, swerve)); // follow trajectory from start to first note to middle
        ArrayList<Event> arrayTwo = new ArrayList<>();
        arrayTwo.add(new SpeakerShootLLEvent(manipulator)); // shoot first note
        arrayTwo.add(new RunIntakeEvent(manipulator)); // intake
        arrayTwo.add(new WaitUntilLoadedEvent(manipulator)); // wait until intook
        arrayTwo.add(new SpeakerShootLLEvent(manipulator)); // shoot second note
        arrayOne.add(new SequentialEvent(arrayTwo)); // sequentially
        events.add(new ParallelEvent(arrayOne)); // parallel
        ArrayList<Event> arrayThree = new ArrayList<>();
        arrayThree.add(new FollowTrajectoryEvent(trajectoryTwo, swerve));
        ArrayList<Event> arrayFour = new ArrayList<>();
        arrayFour.add(new RunIntakeEvent(manipulator));
        arrayFour.add(new WaitUntilLoadedEvent(manipulator));
        arrayFour.add(new SpeakerShootLLEvent(manipulator));
        arrayThree.add(new SequentialEvent(arrayFour));
        events.add(new ParallelEvent(arrayThree));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryThree, swerve));
        events.add(new WaitUntilLoadedEvent(manipulator));
        events.add(new SpeakerShootLLEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryFour, swerve));
        events.add(new WaitUntilLoadedEvent(manipulator));
        events.add(new SpeakerShootLLEvent(manipulator));
        swerve.zeroGyro();
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
