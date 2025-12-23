package frc.excalib.vision;

/**
 * Represents a detected vision target with position and confidence information.
 * 
 * @author Excalib Team
 */
public class VisionTarget {
    private final double x;
    private final double y;
    private final double area;
    private final double confidence;
    private final long timestamp;
    
    /**
     * Creates a new VisionTarget.
     * 
     * @param x horizontal position (typically -1 to 1, 0 is center)
     * @param y vertical position (typically -1 to 1, 0 is center)
     * @param area normalized area (0 to 1)
     * @param confidence detection confidence (0 to 1)
     */
    public VisionTarget(double x, double y, double area, double confidence) {
        this.x = x;
        this.y = y;
        this.area = area;
        this.confidence = confidence;
        this.timestamp = System.currentTimeMillis();
    }
    
    /**
     * Get the horizontal position of the target.
     * @return x position
     */
    public double getX() {
        return x;
    }
    
    /**
     * Get the vertical position of the target.
     * @return y position
     */
    public double getY() {
        return y;
    }
    
    /**
     * Get the normalized area of the target.
     * @return area (0 to 1)
     */
    public double getArea() {
        return area;
    }
    
    /**
     * Get the detection confidence.
     * @return confidence (0 to 1)
     */
    public double getConfidence() {
        return confidence;
    }
    
    /**
     * Get the timestamp when this target was detected.
     * @return timestamp in milliseconds
     */
    public long getTimestamp() {
        return timestamp;
    }
    
    /**
     * Calculate distance from center.
     * @return distance from (0, 0)
     */
    public double getDistanceFromCenter() {
        return Math.sqrt(x * x + y * y);
    }
    
    /**
     * Calculate angle to target from center.
     * @return angle in radians
     */
    public double getAngleFromCenter() {
        return Math.atan2(y, x);
    }
    
    @Override
    public String toString() {
        return String.format("VisionTarget[x=%.3f, y=%.3f, area=%.3f, conf=%.2f]", 
                           x, y, area, confidence);
    }
}
