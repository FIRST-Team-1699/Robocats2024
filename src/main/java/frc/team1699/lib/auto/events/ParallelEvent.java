package frc.team1699.lib.auto.events;

import java.util.ArrayList;

public class ParallelEvent extends Event {
    private ArrayList<Event> events;

    public ParallelEvent(ArrayList<Event> events) {
        this.events = events;
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
        for(Event event : events) {
            if(!event.isFinished()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void finish() {
        for(Event event : events) {
            event.finish();
        }
    }
}