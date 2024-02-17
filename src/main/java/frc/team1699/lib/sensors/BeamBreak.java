package frc.team1699.lib.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak {
    private DigitalInput input;

    public BeamBreak(int channel) {
        this.input = new DigitalInput(channel);
    }

    public boolean isBroken() {
        return !input.get();
    }
}
