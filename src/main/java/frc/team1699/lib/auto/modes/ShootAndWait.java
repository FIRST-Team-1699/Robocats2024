package frc.team1699.lib.auto.modes;

import java.util.ArrayList;
import frc.team1699.lib.auto.events.AimSpeakerEvent;
import frc.team1699.lib.auto.events.Event;
import frc.team1699.lib.auto.events.SpeakerShootLLEvent;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;

public class ShootAndWait extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public ShootAndWait(Manipulator manipulator, Drive swerve) {
        events.add(new AimSpeakerEvent(swerve, manipulator));
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
