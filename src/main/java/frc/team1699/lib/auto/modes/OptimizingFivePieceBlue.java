package frc.team1699.lib.auto.modes;

import java.util.ArrayList;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team1699.lib.auto.events.AimSpeakerEvent;
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

public class OptimizingFivePieceBlue extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public OptimizingFivePieceBlue(Manipulator manipulator, Drive swerve) {
        PathPlannerTrajectory trajectoryOne = PathPlannerPath.fromPathFile("FivePieceOnePlusTwo").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryTwo = PathPlannerPath.fromPathFile("FivePieceThreePlusFour").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        // PathPlannerTrajectory trajectoryThree = PathPlannerPath.fromPathFile("RO4P4").getTrajectory(new ChassisSpeeds(), new Rotation2d());
        PathPlannerTrajectory trajectoryThree = PathPlannerPath.fromPathFile("RO4P5").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(35));
        PathPlannerTrajectory trajectoryFour = PathPlannerPath.fromPathFile("B5P5").getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(10));
        events = new ArrayList<Event>();
        events.add(new SpeakerShootSubEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryOne, swerve));
        events.add(new WaitUntilLoadedEvent(manipulator));
        events.add(new SpeakerShootLLEvent(manipulator));
        // ArrayList<Event> arrayOne = new ArrayList<>();
        // arrayOne.add(new WaitUntilLoadedEvent(manipulator));
        // arrayOne.add(new SpeakerAimLLEvent(manipulator));
        // SequentialEvent sequenceOne = new SequentialEvent(arrayOne);
        // ArrayList<Event> arrayTwo = new ArrayList<>();
        // arrayTwo.add(new FollowTrajectoryEvent(trajectoryTwo, swerve));
        // arrayTwo.add(sequenceOne);
        // events.add(new DeadlineEvent(arrayTwo));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryTwo, swerve));
        events.add(new WaitUntilLoadedEvent(manipulator));
        events.add(new SpeakerShootLLEvent(manipulator));
        events.add(new RunIntakeEvent(manipulator));
        events.add(new FollowTrajectoryEvent(trajectoryThree, swerve));
        events.add(new WaitUntilLoadedEvent(manipulator));
        events.add(new SpeakerShootLLEvent(manipulator));
        ArrayList<Event> arrayOne = new ArrayList<>();
        arrayOne.add(new FollowTrajectoryEvent(trajectoryFour, swerve));
        ArrayList<Event> shootSequence = new ArrayList<>();
        shootSequence.add(new RunIntakeEvent(manipulator));
        shootSequence.add(new WaitUntilLoadedEvent(manipulator));
        shootSequence.add(new SpeakerAimLLEvent(manipulator));
        arrayOne.add(new SequentialEvent(shootSequence));
        events.add(new DeadlineEvent(arrayOne));
        events.add(new AimSpeakerEvent(swerve, manipulator));
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