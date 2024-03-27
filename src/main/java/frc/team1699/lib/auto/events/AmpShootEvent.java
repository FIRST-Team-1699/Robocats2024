package frc.team1699.lib.auto.events;

import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

public class AmpShootEvent extends Event {
    private Manipulator manipulator;

    public AmpShootEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {
        manipulator.setWantedState(ManipulatorStates.AMP_SHOOT);
        manipulator.setWantedState(ManipulatorStates.SHOOTING);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return !manipulator.isLoaded();
    }

    @Override
    public void finish() {
        manipulator.setWantedState(ManipulatorStates.IDLE);
    }
}
