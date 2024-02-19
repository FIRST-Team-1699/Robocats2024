package frc.team1699.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;

import frc.team1699.Constants.ShooterConstants;
import frc.team1699.lib.auto.modes.AutoMode;

// YES I KNOW THIS VIOLATES EVERY CONVENTION
// FROM EVERY OTHER SUBSYSTEM, LET ME COOK.
public class Shooter {
    private boolean isAtSpeed = false;
    private double topSetpoint = 0.0;
    private double bottomSetpoint = 0.0;

    private TalonFX topFX;
    private TalonFX bottomFX;
    private TalonFXConfiguration configs;

    private VelocityVoltage motorRequest;

    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kV = 0.11;

    private Orchestra orchestra;
    
    public Shooter() {
        motorRequest = new VelocityVoltage(0.0);
        configs = new TalonFXConfiguration();
        configs.Slot0.kP = kP;
        configs.Slot0.kI = kI;
        configs.Slot0.kD = kD;
        configs.Slot0.kV = kV;
        configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;
        topFX = new TalonFX(ShooterConstants.kTopMotorID);
        topFX.getConfigurator().apply(configs);
        bottomFX = new TalonFX(ShooterConstants.kBottomMotorID);
        bottomFX.getConfigurator().apply(configs);

        orchestra = new Orchestra();
        orchestra.addInstrument(topFX);
        orchestra.addInstrument(bottomFX);
        orchestra.loadMusic("stacy.chrp");
    }

    public void startOrchestra() {
        if(orchestra.isPlaying()) {
            return;
        }
        orchestra.play();
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
        if(Math.abs(topFX.getVelocity().getValueAsDouble() - topSetpoint) >= 5) {
            return false;
        }
        if(Math.abs(bottomFX.getVelocity().getValueAsDouble() - bottomSetpoint) >= 5) {
            return false;
        }
        return true;
    }

    public void update() {
        // check if we are at speed and anything else i think of later
        topFX.setControl(motorRequest.withVelocity(topSetpoint));
        bottomFX.setControl(motorRequest.withVelocity(bottomSetpoint));
    }
}