package frc.team1699.lib.auto.events;

import frc.team1699.subsystems.Intake;
import frc.team1699.subsystems.Intake.IntakeStates;

public class StopIntakeEvent extends Event {
    private Intake intake;

    public StopIntakeEvent(Intake intake) {
        this.intake = intake;
    }
    
    @Override
    public void initialize() {
        intake.setWantedState(IntakeStates.IDLE);
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
