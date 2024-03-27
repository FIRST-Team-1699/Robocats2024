package frc.team1699.lib.leds.colors;

public class HSVColor {
    private int hue, saturation;
    private final int value = 30;

    public HSVColor() {
        hue = 0;
        saturation = 0;
    }

    public HSVColor(int h, int s) {
        hue = h;
        saturation = s;
    }

    public int getHue() {
        return hue;
    }

    public void setHue(int h) {
        hue = h;
    }

    public int getSaturation() {
        return saturation;
    }

    public void setSaturation(int s) {
        saturation = s;
    }
    
    public int getValue() {
        return value;
    }

    public boolean equals(HSVColor compareColor) {
        System.out.println("already at that color");
        System.out.println(getHue() == compareColor.getHue());
        return getHue() == compareColor.getHue();
    }
}
