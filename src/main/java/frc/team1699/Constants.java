package frc.team1699;
public class Constants {
    // The DI/O port where the LED strip is plugged in
    public static final int kLEDPort = 0;
    public static class SwerveConstants {
        public static final double kDeadband = .2;
        public static final double kMaxspeed = Units.feetToMeters(15,1);
        public static final double kMaxRetationalSpeed = Units.feetToMeters(10); // DOES THIS NEED TO BE DIFFRENT THAN MAX SPEED? 
    }
}