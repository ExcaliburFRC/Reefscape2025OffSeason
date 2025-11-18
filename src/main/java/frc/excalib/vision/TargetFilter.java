package frc.excalib.vision;

import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;

/**
 * Common filters for vision target selection.
 * 
 * @author Excalib Team
 */
public class TargetFilter {
    
    /**
     * Filter targets by minimum area.
     * @param minArea minimum normalized area (0 to 1)
     * @return predicate for filtering
     */
    public static Predicate<VisionTarget> minArea(double minArea) {
        return target -> target.getArea() >= minArea;
    }
    
    /**
     * Filter targets by minimum confidence.
     * @param minConfidence minimum confidence (0 to 1)
     * @return predicate for filtering
     */
    public static Predicate<VisionTarget> minConfidence(double minConfidence) {
        return target -> target.getConfidence() >= minConfidence;
    }
    
    /**
     * Filter targets within a horizontal range.
     * @param minX minimum x position
     * @param maxX maximum x position
     * @return predicate for filtering
     */
    public static Predicate<VisionTarget> inXRange(double minX, double maxX) {
        return target -> target.getX() >= minX && target.getX() <= maxX;
    }
    
    /**
     * Filter targets within a vertical range.
     * @param minY minimum y position
     * @param maxY maximum y position
     * @return predicate for filtering
     */
    public static Predicate<VisionTarget> inYRange(double minY, double maxY) {
        return target -> target.getY() >= minY && target.getY() <= maxY;
    }
    
    /**
     * Select the target closest to center.
     * @param targets list of targets
     * @return the closest target, or null if list is empty
     */
    public static VisionTarget selectClosestToCenter(List<VisionTarget> targets) {
        return targets.stream()
            .min((a, b) -> Double.compare(a.getDistanceFromCenter(), b.getDistanceFromCenter()))
            .orElse(null);
    }
    
    /**
     * Select the largest target by area.
     * @param targets list of targets
     * @return the largest target, or null if list is empty
     */
    public static VisionTarget selectLargest(List<VisionTarget> targets) {
        return targets.stream()
            .max((a, b) -> Double.compare(a.getArea(), b.getArea()))
            .orElse(null);
    }
    
    /**
     * Select the most confident target.
     * @param targets list of targets
     * @return the most confident target, or null if list is empty
     */
    public static VisionTarget selectMostConfident(List<VisionTarget> targets) {
        return targets.stream()
            .max((a, b) -> Double.compare(a.getConfidence(), b.getConfidence()))
            .orElse(null);
    }
    
    /**
     * Apply multiple filters to a target list.
     * @param targets input targets
     * @param filters filters to apply
     * @return filtered targets
     */
    @SafeVarargs
    public static List<VisionTarget> applyFilters(List<VisionTarget> targets, 
                                                   Predicate<VisionTarget>... filters) {
        List<VisionTarget> result = targets;
        for (Predicate<VisionTarget> filter : filters) {
            result = result.stream()
                .filter(filter)
                .collect(Collectors.toList());
        }
        return result;
    }
}
