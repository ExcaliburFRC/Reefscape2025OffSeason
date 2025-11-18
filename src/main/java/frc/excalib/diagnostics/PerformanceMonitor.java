package frc.excalib.diagnostics;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Monitors robot performance metrics like CPU usage, memory, and loop timing.
 * Helps identify bottlenecks and prevent performance issues during matches.
 * 
 * Example:
 * <pre>
 * PerformanceMonitor monitor = PerformanceMonitor.getInstance();
 * monitor.startCycle();
 * // ... robot code ...
 * monitor.endCycle();
 * </pre>
 * 
 * @author Excalib Team
 */
public class PerformanceMonitor {
    private static PerformanceMonitor instance;
    
    private double cycleStartTime;
    private double maxCycleTime = 0.0;
    private double minCycleTime = Double.MAX_VALUE;
    private double totalCycleTime = 0.0;
    private long cycleCount = 0;
    
    private boolean cpuWatchEnabled = false;
    private boolean memoryWatchEnabled = false;
    private boolean canWatchEnabled = false;
    
    private PerformanceMonitor() {}
    
    /**
     * Get the singleton instance.
     * @return the PerformanceMonitor instance
     */
    public static PerformanceMonitor getInstance() {
        if (instance == null) {
            instance = new PerformanceMonitor();
        }
        return instance;
    }
    
    /**
     * Start a performance monitoring cycle.
     * Call this at the beginning of your periodic methods.
     */
    public void startCycle() {
        cycleStartTime = Timer.getFPGATimestamp();
    }
    
    /**
     * End a performance monitoring cycle and record metrics.
     * Call this at the end of your periodic methods.
     */
    public void endCycle() {
        double cycleTime = Timer.getFPGATimestamp() - cycleStartTime;
        
        maxCycleTime = Math.max(maxCycleTime, cycleTime);
        minCycleTime = Math.min(minCycleTime, cycleTime);
        totalCycleTime += cycleTime;
        cycleCount++;
        
        SmartDashboard.putNumber("Performance/CycleTime_ms", cycleTime * 1000.0);
        SmartDashboard.putNumber("Performance/AvgCycleTime_ms", (totalCycleTime / cycleCount) * 1000.0);
        SmartDashboard.putNumber("Performance/MaxCycleTime_ms", maxCycleTime * 1000.0);
        
        if (cycleTime > 0.02) { // 20ms threshold (robot loop is 20ms)
            System.err.println(String.format(
                "[PERFORMANCE WARNING] Cycle time exceeded 20ms: %.2f ms",
                cycleTime * 1000.0
            ));
        }
    }
    
    /**
     * Enable CPU usage monitoring.
     * @return this for method chaining
     */
    public PerformanceMonitor watchCPU() {
        cpuWatchEnabled = true;
        return this;
    }
    
    /**
     * Enable memory usage monitoring.
     * @return this for method chaining
     */
    public PerformanceMonitor watchMemory() {
        memoryWatchEnabled = true;
        return this;
    }
    
    /**
     * Enable CAN bus utilization monitoring.
     * @return this for method chaining
     */
    public PerformanceMonitor watchCANUtilization() {
        canWatchEnabled = true;
        return this;
    }
    
    /**
     * Update all enabled performance metrics.
     * Call this periodically to update dashboard values.
     */
    public void update() {
        if (cpuWatchEnabled) {
            // Note: CPU usage not directly available in WPILib, using proxy metrics
            SmartDashboard.putNumber("Performance/CycleCount", cycleCount);
        }
        
        if (memoryWatchEnabled) {
            Runtime runtime = Runtime.getRuntime();
            long totalMemory = runtime.totalMemory();
            long freeMemory = runtime.freeMemory();
            long usedMemory = totalMemory - freeMemory;
            
            SmartDashboard.putNumber("Performance/MemoryUsed_MB", usedMemory / (1024.0 * 1024.0));
            SmartDashboard.putNumber("Performance/MemoryTotal_MB", totalMemory / (1024.0 * 1024.0));
            SmartDashboard.putNumber("Performance/MemoryPercent", (usedMemory * 100.0) / totalMemory);
        }
        
        if (canWatchEnabled) {
            SmartDashboard.putNumber("Performance/CANUtilization", 
                                    RobotController.getCANStatus().percentBusUtilization);
        }
        
        // Always monitor battery
        SmartDashboard.putNumber("Performance/BatteryVoltage", RobotController.getBatteryVoltage());
    }
    
    /**
     * Get the maximum recorded cycle time.
     * @return max cycle time in seconds
     */
    public double getMaxCycleTime() {
        return maxCycleTime;
    }
    
    /**
     * Get the minimum recorded cycle time.
     * @return min cycle time in seconds
     */
    public double getMinCycleTime() {
        return minCycleTime;
    }
    
    /**
     * Get the average cycle time.
     * @return average cycle time in seconds
     */
    public double getAverageCycleTime() {
        return cycleCount > 0 ? totalCycleTime / cycleCount : 0.0;
    }
    
    /**
     * Reset all performance statistics.
     */
    public void reset() {
        maxCycleTime = 0.0;
        minCycleTime = Double.MAX_VALUE;
        totalCycleTime = 0.0;
        cycleCount = 0;
    }
}
