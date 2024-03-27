package frc.team1699.lib.auto.events;

import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

public class RunIntakeEvent extends Event {
    private Manipulator manipulator;

    public RunIntakeEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
    }
    
    @Override
    public void initialize() {
        manipulator.setWantedState(ManipulatorStates.INTAKING);
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
