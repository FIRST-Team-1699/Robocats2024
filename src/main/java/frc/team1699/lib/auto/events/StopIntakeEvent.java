package frc.team1699.lib.auto.events;

import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

public class StopIntakeEvent extends Event {
    private Manipulator manipulator;

    public StopIntakeEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
    }
    
    @Override
    public void initialize() {
        manipulator.setWantedState(ManipulatorStates.IDLE);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void finish() {}
    
}
