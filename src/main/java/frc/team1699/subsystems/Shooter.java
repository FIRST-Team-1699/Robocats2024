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
    private boolean isAtSpeed = true;
    private double setpoint = 0.0;

    private TalonFX topFX;
    private TalonFX bottomFX;
    private TalonFXConfiguration configs;

    private VoltageOut motorRequest;

    private PIDController topPID;
    private PIDController bottomPID;

    private final double kTopP = 0.0;
    private final double kTopI = 0.0;
    private final double kTopD = 0.0;

    private final double kBottomP = 0.0;
    private final double kBottomI = 0.0;
    private final double kBottomD = 0.0;

    private Orchestra orchestra;
    
    public Shooter() {
        motorRequest = new VoltageOut(0);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        topFX = new TalonFX(ShooterConstants.kTopMotorID);
        bottomFX = new TalonFX(ShooterConstants.kBottomMotorID);

        // PIDs
        topPID = new PIDController(kTopP, kTopI, kTopD);
        bottomPID = new PIDController(kBottomP, kBottomI, kBottomD);

        orchestra = new Orchestra();
        orchestra.addInstrument(topFX);
        orchestra.addInstrument(bottomFX);
        orchestra.loadMusic(null);
    }

    public void startOrchestra() {
        if(orchestra.isPlaying()) {
            return;
        }
        orchestra.play();
    }

    public void setSpeed(double speed) {
        // TODO give the motors a new setpoint
        // is at speed becomes false
        topPID.setSetpoint(speed);
        bottomPID.setSetpoint(speed);

    }

    public boolean atSpeed() {
        // returns the value of isatspeed
        if(topPID.atSetpoint() && bottomPID.atSetpoint()) {
            return true;
        }
        return false;
    }

    public void update() {
        // check if we are at speed and anything else i think of later
        topFX.setControl(motorRequest.withOutput(topPID.calculate(topFX.getVelocity().getValueAsDouble())));
        bottomFX.setControl(motorRequest.withOutput(bottomPID.calculate(bottomFX.getVelocity().getValueAsDouble())));

        
    }
}
