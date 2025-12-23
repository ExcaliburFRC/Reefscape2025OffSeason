package frc.excalib.additional_utilities;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Pre-built LED patterns for common robot states.
 * Makes it easy to provide visual feedback to drivers and pit crew.
 * 
 * Example:
 * <pre>
 * LEDPattern pattern = LEDPattern.rainbow();
 * pattern.apply(ledBuffer, Timer.getFPGATimestamp());
 * leds.setData(ledBuffer);
 * </pre>
 * 
 * @author Excalib Team
 */
public abstract class LEDPattern {
    
    /**
     * Apply this pattern to an LED buffer.
     * @param buffer the LED buffer to modify
     * @param timestamp current time for animation
     */
    public abstract void apply(AddressableLEDBuffer buffer, double timestamp);
    
    /**
     * Solid color pattern.
     */
    public static LEDPattern solid(Color color) {
        return new LEDPattern() {
            @Override
            public void apply(AddressableLEDBuffer buffer, double timestamp) {
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setLED(i, color);
                }
            }
        };
    }
    
    /**
     * Blinking pattern.
     * @param color the color to blink
     * @param frequency blinks per second
     */
    public static LEDPattern blink(Color color, double frequency) {
        return new LEDPattern() {
            @Override
            public void apply(AddressableLEDBuffer buffer, double timestamp) {
                boolean on = (timestamp * frequency) % 1.0 < 0.5;
                Color c = on ? color : Color.kBlack;
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setLED(i, c);
                }
            }
        };
    }
    
    /**
     * Rainbow pattern that cycles through colors.
     */
    public static LEDPattern rainbow() {
        return new LEDPattern() {
            @Override
            public void apply(AddressableLEDBuffer buffer, double timestamp) {
                int length = buffer.getLength();
                for (int i = 0; i < length; i++) {
                    int hue = (int) ((timestamp * 180 + (i * 180.0 / length)) % 180);
                    buffer.setHSV(i, hue, 255, 128);
                }
            }
        };
    }
    
    /**
     * Chase pattern with a moving dot.
     * @param color the color of the dot
     * @param speed speed in LEDs per second
     */
    public static LEDPattern chase(Color color, double speed) {
        return new LEDPattern() {
            @Override
            public void apply(AddressableLEDBuffer buffer, double timestamp) {
                int length = buffer.getLength();
                int position = (int) (timestamp * speed) % length;
                
                for (int i = 0; i < length; i++) {
                    if (i == position) {
                        buffer.setLED(i, color);
                    } else {
                        buffer.setLED(i, Color.kBlack);
                    }
                }
            }
        };
    }
    
    /**
     * Strobe pattern (rapid flashing).
     * @param color the color to strobe
     */
    public static LEDPattern strobe(Color color) {
        return blink(color, 10.0); // 10 Hz strobe
    }
    
    /**
     * Breathe pattern (smooth fade in/out).
     * @param color the base color
     * @param frequency breaths per second
     */
    public static LEDPattern breathe(Color color, double frequency) {
        return new LEDPattern() {
            @Override
            public void apply(AddressableLEDBuffer buffer, double timestamp) {
                double phase = (timestamp * frequency) % 1.0;
                double brightness = Math.sin(phase * Math.PI);
                
                int r = (int) (color.red * brightness * 255);
                int g = (int) (color.green * brightness * 255);
                int b = (int) (color.blue * brightness * 255);
                
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, r, g, b);
                }
            }
        };
    }
    
    /**
     * Two-color alternating pattern.
     */
    public static LEDPattern alternate(Color color1, Color color2) {
        return new LEDPattern() {
            @Override
            public void apply(AddressableLEDBuffer buffer, double timestamp) {
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setLED(i, i % 2 == 0 ? color1 : color2);
                }
            }
        };
    }
    
    /**
     * Progress bar pattern showing a percentage.
     * @param percent completion percentage (0.0 to 1.0)
     * @param filledColor color for filled portion
     * @param emptyColor color for empty portion
     */
    public static LEDPattern progressBar(double percent, Color filledColor, Color emptyColor) {
        return new LEDPattern() {
            @Override
            public void apply(AddressableLEDBuffer buffer, double timestamp) {
                int length = buffer.getLength();
                int filledLength = (int) (length * Math.min(1.0, Math.max(0.0, percent)));
                
                for (int i = 0; i < length; i++) {
                    buffer.setLED(i, i < filledLength ? filledColor : emptyColor);
                }
            }
        };
    }
    
    // Common robot state patterns
    
    /** Pattern for when robot is disabled */
    public static final LEDPattern DISABLED = blink(Color.kOrange, 0.5);
    
    /** Pattern for when robot has a game piece */
    public static final LEDPattern HAS_GAME_PIECE = solid(Color.kGreen);
    
    /** Pattern for when robot is aligned to target */
    public static final LEDPattern ALIGNED = strobe(Color.kGreen);
    
    /** Pattern for error/fault state */
    public static final LEDPattern ERROR = strobe(Color.kRed);
    
    /** Pattern for autonomous mode */
    public static final LEDPattern AUTO = chase(Color.kBlue, 20.0);
    
    /** Pattern for teleop mode */
    public static final LEDPattern TELEOP = solid(Color.kPurple);
    
    /** Pattern for climbing */
    public static final LEDPattern CLIMBING = breathe(Color.kYellow, 1.0);
}
