package frc.team1699.lib.auto.events;

import java.util.ArrayList;

public class DeadlineEvent extends Event {
    private Event priorityEvent;
    private ArrayList<Event> events;

    public DeadlineEvent(ArrayList<Event> events) {
        this.events = events;
        this.priorityEvent = events.get(0);
    }

    @Override
    public void initialize() {
        for(Event event : events) {
            event.initialize();
        }
    }

    @Override
    public void update() {
        for(Event event : events) {
            event.update();
        }
    }

    @Override
    public boolean isFinished() {
        if(priorityEvent.isFinished()) {
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