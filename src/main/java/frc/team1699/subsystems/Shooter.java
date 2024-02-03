package frc.team1699.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.team1699.Constants.ShooterConstants;

// YES I KNOW THIS VIOLATES EVERY CONVENTION
// FROM EVERY OTHER SUBSYSTEM, LET ME COOK.
public class Shooter {
    private boolean isAtSpeed = true;
    private double setpoint = 0.0;

    private TalonFX topFX;
    private TalonFX bottomFX;
    private TalonFXConfiguration configs;
    
    public Shooter() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        topFX = new TalonFX(ShooterConstants.kTopMotorID);
        bottomFX = new TalonFX(ShooterConstants.kBottomMotorID);
    }
}
