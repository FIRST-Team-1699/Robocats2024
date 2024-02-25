package frc.team1699.lib.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.team1699.lib.leds.colors.HSVColor;
import frc.team1699.lib.leds.colors.Red;
import frc.team1699.lib.leds.colors.Yellow;

public class LEDController {
    private int rainbowFirstPixelHue = 50;
    private int yellowBlink = 0;

    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private int ledLength;

    private HSVColor currentColor;

    public LEDController(int ledLength, int port) {
        this.ledLength = ledLength;
        leds = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        leds.setLength(ledLength);
    }

    public void start() {
        leds.start();
    }

    public void stop() {
        leds.stop();
    }

    
    public void rainbow() {
        for (int i = 0; i < ledLength; i++) {
            final int hue = (rainbowFirstPixelHue + (i * 180 / ledLength)) % 180;
            ledBuffer.setHSV(i, hue, 255, 50);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
        leds.setData(ledBuffer);
    }

    public void solidColor(HSVColor color) {
        for(int i = 0; i < ledLength; i++) {
            ledBuffer.setHSV(i, color.getHue(), color.getSaturation(), color.getValue());
        }
        leds.setData(ledBuffer);
        currentColor = color;
    }

    public void alternateColors(HSVColor colorOne, HSVColor colorTwo) {
        for(int i = 0; i < ledLength; i++) {
            if(i%2 == 0) {
                ledBuffer.setHSV(i, colorOne.getHue(), colorOne.getSaturation(), colorOne.getValue());
            } else {
                ledBuffer.setHSV(i, colorTwo.getHue(), colorTwo.getSaturation(), colorTwo.getValue());
            }
        }
        leds.setData(ledBuffer);
    }

    public void blinkYellow() {
        yellowBlink %= 10;
        if(yellowBlink == 0) {
            solidColor(new Yellow());
        } else if(yellowBlink == 5) {
            solidColor(new Red());
        }
    }

    public HSVColor getColor() {
        return currentColor;
    }
}