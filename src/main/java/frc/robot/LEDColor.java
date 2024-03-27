package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Utility class for colors based on WPILib {@link Color}
 */
public class LEDColor extends Color {

    public double red, green, blue;
    public double hue, saturation, value;

    public LEDColor(Color color) {
        super(color.red, color.green, color.blue);
        red = color.red;
        green = color.green;
        blue = color.blue;
        calcHSV();
    }

    /**
     * Constructs an LEDColor.
     * @param red 0-1
     * @param green 0-1
     * @param blue 0-1
     */
    public LEDColor(double red, double green, double blue) {
        super(red, green, blue);
        this.red=red;
        this.green=green;
        this.blue=blue;
        calcHSV();
    }
    
    /**
     * Constructs an LEDColor.
     * @param red 0-255
     * @param green 0-255
     * @param blue 0-255
     */
    public LEDColor(int red, int green, int blue) {
        super(red, green, blue);
        this.red=red/255.0;
        this.green=green/255.0;
        this.blue=blue/255.0;
        calcHSV();
    }

    @Override
    public boolean equals(Object other) {
        if (super.equals(other)) {
            return true;
        }
        
        if (other == null || getClass() != other.getClass()) {
            return false;
        }

        LEDColor color = (LEDColor) other;
        return Double.compare(hue, color.hue) == 0
        && Double.compare(saturation, color.saturation) == 0
        && Double.compare(green, blue) == 0;
    }

    public void setRGB(double r, double g, double b) {
        red=r;
        green=g;
        blue=b;
        calcHSV();
    }

    public void setRGB(int r, int g, int b) {
        red=r;
        green=g;
        blue=b;
        calcHSV();
    }

    public void setHSV(double h, double s, double v) {
        hue=h;
        saturation=s;
        value=v;
        calcRGB();
    }

    public void setHSV(int h, int s, int v) {
        hue=h;
        saturation=s;
        value=v;
        calcRGB();
    }

    /**
     * Linearly interpolates between two colors over time.
     * @param a First {@link Color}
     * @param b Second {@link Color}
     * @param t Time, in seconds
     * @return The calculated color at t seconds
     */
    public static LEDColor lerpRGB(Color a, Color b, double t) {
        double time = MathUtil.clamp(t, 0, 1);
        return new LEDColor(
            a.red + ((b.red - a.red) * time),
            a.green + ((b.green - a.green) * time),
            a.blue + ((b.blue - a.blue) * time)
        );
    }

    /**
     * Linearly interpolates between two colors over time.
     * @param a First {@link LEDColor}
     * @param b Second {@link LEDColor}
     * @param t Time
     * @return The product color.
     */
    public static LEDColor lerpHSV(LEDColor a, LEDColor b, double t) {
        double time = MathUtil.clamp(t, 0, 1);

        LEDColor out = new LEDColor(Color.kBlack);
        out.setHSV(
            a.hue + time * (b.hue - a.hue), 
            a.saturation + time * (b.saturation - a.saturation), 
            a.value + time * (b.value - a.value)
        );
        return out;
    }
    
    /**
     * Calculates HSV from RGB
     */
    void calcHSV() {
        double rprime = red / 255;
        double gprime = green / 255;
        double bprime = blue / 255;

        double cMax = Math.max(rprime, Math.max(gprime, bprime));
        double cMin = Math.min(rprime, Math.min(gprime, bprime));

        double delta = cMax - cMin;

        value = cMax * 255;

        saturation = cMax == 0 ? 0 : (delta/cMax);

        if (delta == 0) 
            hue = 0;
        else if (cMax == rprime)
            hue = 30 * (((gprime - bprime)/delta) % 6);
        else if (cMax == gprime)
            hue = 30 * (((bprime - rprime)/delta) + 2);
        else if (cMax == bprime)
            hue = 30 * (((rprime - gprime)/delta) + 4);
    }

    /**
     * Calculates RGB from HSV
     */
    void calcRGB() {
        double cMax = value;
        double cMin = cMax * (1 - (saturation/255));
        double z = (cMax - cMin) * (1 - Math.abs((hue / 30) % 2 - 1));

        if (hue < 30) {
            red = cMax;
            green = z + cMin;
            blue = cMin;
        } else if (30 <= hue && hue < 60) {
            red = z + cMin;
            green = cMax;
            blue = cMin;
        } else if (60 <= hue && hue < 90) {
            red = cMin;
            green = cMax;
            blue = z + cMin;
        } else if (90 <= hue && hue < 120) {
            red = cMin;
            green = z + cMin;
            blue = cMax;
        } else if (120 <= hue && hue < 150) {
            red = z + cMin;
            green = cMin;
            blue = cMax;
        } else if (150 <= hue && hue < 180) {
            red = cMax;
            green = cMin;
            blue = z + cMin;
        }
    }
}
