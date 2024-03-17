package frc.team1699.lib.leds;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.team1699.lib.leds.colors.Blue;
import frc.team1699.lib.leds.colors.Green;
import frc.team1699.lib.leds.colors.HSVColor;
import frc.team1699.lib.leds.colors.Red;
import frc.team1699.lib.leds.colors.Yellow;
import frc.team1699.subsystems.Drive;
import frc.team1699.subsystems.Manipulator;
import frc.team1699.subsystems.Vision;

public class LEDController {
    private int rainbowFirstPixelHue = 50;      
    private int yellowBlink = 0;
    private HSVColor lastAimColor = new Red();

    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private int ledLength;
    private ArrayList<LEDStates> ledStateBuffer;
    private LEDStates ledState;
    private Drive swerve;
    private Manipulator manipulator;

    public LEDController(int ledLength, int port, Drive swerve, Manipulator manipulator) {
        this.ledLength = ledLength;
        this.leds = new AddressableLED(port);
        this.ledBuffer = new AddressableLEDBuffer(ledLength);
        leds.setLength(ledLength);
        leds.start();
        this.ledStateBuffer = new ArrayList<>();
        this.swerve = swerve;
        this.manipulator = manipulator;
    }
    
    // private void rainbow() {
    //     for (int i = 0; i < ledLength; i++) {
    //         final int hue = (rainbowFirstPixelHue + (i * 180 / ledLength)) % 180;
    //         ledBuffer.setHSV(i, hue, 255, 50);
    //     }
    //     rainbowFirstPixelHue += 3;
    //     rainbowFirstPixelHue %= 180;
    //     leds.setData(ledBuffer);
    // }

    public void solidColor(HSVColor color) {
        for(int i = 0; i < ledLength; i++) {
            ledBuffer.setHSV(i, color.getHue(), color.getSaturation(), color.getValue());
        }
        leds.setData(ledBuffer);
    }

    // private void alternateColors(HSVColor colorOne, HSVColor colorTwo) {
    //     for(int i = 0; i < ledLength; i++) {
    //         if(i % 2 == 0) {
    //             ledBuffer.setHSV(i, colorOne.getHue(), colorOne.getSaturation(), colorOne.getValue());
    //         } else {
    //             ledBuffer.setHSV(i, colorTwo.getHue(), colorTwo.getSaturation(), colorTwo.getValue());
    //         }
    //     }
    //     leds.setData(ledBuffer);
    // }

    private void blinkYellowRed() {
        yellowBlink %= 10;
        if(yellowBlink == 0) {
            solidColor(new Yellow());
        } else if(yellowBlink == 5) {
            solidColor(new Red());
        }
        yellowBlink++;
    }

    public void update() {
        if(ledStateBuffer.size() > 0) {
            handleStateTransition(getPriorityState(ledStateBuffer));
            ledStateBuffer.clear();
        }
        switch(ledState) {
            case AIMING:
                if(swerve.headingAimed() && manipulator.pivotAtPose() && manipulator.shooterAtSpeed()) {
                    lastAimColor = new Green();
                    solidColor(lastAimColor);
                } else if(!Vision.getInstance().hasTargetInView()) {
                    if(!lastAimColor.equals(new Red())) {
                        lastAimColor = new Red();
                        solidColor(lastAimColor);
                    }
                } else {
                    if(!lastAimColor.equals(new Blue())) {
                        lastAimColor = new Blue();
                        solidColor(lastAimColor);
                    }   
                }
                break;
            case AMPLIFY:
                blinkYellowRed();
                break;
            case IDLE:
                if(manipulator.intakeLoaded() || manipulator.isLoaded()) {
                    solidColor(new Yellow());
                } else {
                    solidColor(new Blue());
                }
                break;
            default:
                break;
        }
    }

    private void handleStateTransition(LEDStates newState) {
        switch(newState) {
            case AIMING:
                break;
            case AMPLIFY:
                break;
            case IDLE:
                break;
            default:
                break;

        }
        ledState = newState;
    }

    private LEDStates getPriorityState(ArrayList<LEDStates> buffer) {
        if(buffer.contains(LEDStates.AMPLIFY)) {
            return LEDStates.AMPLIFY;
        } else if(buffer.contains(LEDStates.AIMING)) {
            return LEDStates.AIMING;
        }
        return LEDStates.IDLE;
    }

    public void addState(LEDStates state) {
        ledStateBuffer.add(state);
    }

    public enum LEDStates {
        AMPLIFY,
        AIMING,
        IDLE
    }
}