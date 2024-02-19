package frc.team1699.lib.auto.events;

import frc.team1699.subsystems.Manipulator;

public class WaitUntilLoadedEvent extends Event {
    private Manipulator manipulator;

    public WaitUntilLoadedEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if(manipulator.isLoaded()) {
            return true;
        }
        return false;
    }

    @Override
    public void finish() {}
}