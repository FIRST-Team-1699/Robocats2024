package frc.team1699.lib.auto.events;

import java.util.ArrayList;

public class SequentialEvent extends Event {
    private ArrayList<Event> events;
    private int i;

    public SequentialEvent(ArrayList<Event> events) {
        this.events = events;
        this.i = 0;
    }

    @Override
    public void initialize() {
        events.get(i).initialize();
    }

    @Override
    public void update() {
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
    public void finish() {
        for(Event event : events) {
            event.finish();
        }
    }
}