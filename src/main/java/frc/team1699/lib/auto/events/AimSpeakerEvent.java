package frc.team1699.lib.auto.events;

import edu.wpi.first.wpilibj.Timer;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Drive.DriveState;
import frc.team1699.subsystems.Manipulator.ManipulatorStates;

public class AimSpeakerEvent extends Event {
    private Drive swerve;
    private Manipulator manipulator;
    private Timer timer;

    public AimSpeakerEvent(Drive swerve, Manipulator manipulator) {
        this.swerve = swerve;
        this.manipulator = manipulator;
        this.timer = new Timer();
        timer.stop();
        timer.reset();
    }

    @Override
    public void initialize() {
        swerve.setWantedState(DriveState.TELEOP_SPEAKER_TRACK);
        manipulator.setWantedState(ManipulatorStates.SPEAKER_LL_SHOOT);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return swerve.headingAimed();
    }

    @Override
    public void finish() {}
}
