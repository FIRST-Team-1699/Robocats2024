package frc.team1699.lib.auto.events;

import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

public class SpeakerAimLLEvent extends Event {
    private Manipulator manipulator;

    public SpeakerAimLLEvent(Manipulator manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {}

    @Override
    public void update() {
        manipulator.setWantedState(ManipulatorStates.SPEAKER_LL_SHOOT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void finish() {}
}
