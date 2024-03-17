package frc.team1699.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.team1699.Constants.ShooterConstants;

// YES I KNOW THIS VIOLATES EVERY CONVENTION
// FROM EVERY OTHER SUBSYSTEM, LET ME COOK.
public class Shooter {
    private double topSetpoint = 0.0;
    private double bottomSetpoint = 0.0;

    private TalonFX topFX;
    private TalonFX bottomFX;
    private TalonFXConfiguration configs;

    private VelocityVoltage motorRequest;
    
    public Shooter() {
        motorRequest = new VelocityVoltage(0.0);
        configs = new TalonFXConfiguration();
        configs.Slot0.kP = ShooterConstants.kP;
        configs.Slot0.kI = ShooterConstants.kI;
        configs.Slot0.kD = ShooterConstants.kD;
        configs.Slot0.kV = ShooterConstants.kV;
        configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;
        topFX = new TalonFX(ShooterConstants.kTopMotorID);
        topFX.getConfigurator().apply(configs);
        bottomFX = new TalonFX(ShooterConstants.kBottomMotorID);
        bottomFX.getConfigurator().apply(configs);
    }

    public void setSpeed(double speed) {
        topSetpoint = speed;
        bottomSetpoint = speed;
    }

    public void setSeparateSpeeds(double topSpeed, double bottomSpeed) {
        topSetpoint = topSpeed;
        bottomSetpoint = bottomSpeed;
    }

    public boolean atSpeed() {
        if(Math.abs(topFX.getVelocity().getValueAsDouble() - topSetpoint) >= ShooterConstants.kTolerance || Math.abs(bottomFX.getVelocity().getValueAsDouble()) - bottomSetpoint >= ShooterConstants.kTolerance) {
            return false;
        }
        return true;
    }

    public void update() {
        topFX.setControl(motorRequest.withVelocity(topSetpoint));
        bottomFX.setControl(motorRequest.withVelocity(bottomSetpoint));
        // System.out.println("TOP ERROR: " + topFX.getClosedLoopError());
        // System.out.println("BOTTOM ERROR: " + bottomFX.getClosedLoopError());
    }
}