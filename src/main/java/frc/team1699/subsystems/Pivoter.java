package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.team1699.Constants.PivoterConstants;

public class Pivoter {
    private CANSparkMax pivotMotor;
    private SparkAbsoluteEncoder pivotEncoder;
    private SparkPIDController pivotController;
    private double setpoint;

    public Pivoter() {
        pivotMotor = new CANSparkMax(PivoterConstants.kMotorID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setPositionConversionFactor(360.0);
        pivotEncoder.setZeroOffset(PivoterConstants.kEncoderOffset);
        pivotController = pivotMotor.getPIDController();
        pivotController.setFeedbackDevice(pivotEncoder);
        pivotController.setP(PivoterConstants.kP);
        pivotController.setI(PivoterConstants.kI);
        pivotController.setD(PivoterConstants.kD);
    }

    public void setAngle(double angle) {
        pivotController.setReference(angle, ControlType.kPosition);
    }

    public boolean isAtAngle() {
        if(Math.abs(pivotEncoder.getPosition() - setpoint) <= PivoterConstants.kTolerance) {
            return true;
        }
        return false;
    }

    public void printEncoderValue() {
        System.out.println(pivotEncoder.getPosition());
    }
}