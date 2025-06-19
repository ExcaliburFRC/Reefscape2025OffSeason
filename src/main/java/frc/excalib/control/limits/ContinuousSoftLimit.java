package frc.excalib.control.limits;

import java.util.function.DoubleSupplier;

public class ContinuousSoftLimit extends SoftLimit {
    /**
     * A constructor that takes two DoubleSuppliers representing the dynamic limits:
     *
     * @param minLimit the minimal limit of the represented range
     * @param maxLimit the maximal limit of the represented range
     */
    public ContinuousSoftLimit(DoubleSupplier minLimit, DoubleSupplier maxLimit) {
        super(minLimit, maxLimit);
    }

    public double getSetpoint(double measurement, double wantedSetpoint) {
        double upperSetPoint, lowerSetPoint;
        if (wantedSetpoint > measurement) {
            upperSetPoint = wantedSetpoint;
            while ((upperSetPoint - 2 * Math.PI) > measurement) {
                upperSetPoint -= 2 * Math.PI;
            }
            lowerSetPoint = upperSetPoint - 2 * Math.PI;
        } else if (wantedSetpoint < measurement) {
            lowerSetPoint = wantedSetpoint;
            while ((lowerSetPoint + 2 * Math.PI) < measurement) {
                lowerSetPoint += 2 * Math.PI;
            }
            upperSetPoint = lowerSetPoint + 2 * Math.PI;
        } else {
            return wantedSetpoint;
        }
        if (upperSetPoint > super.getMaxLimit()) {
            return lowerSetPoint;
        } else if (lowerSetPoint < super.getMinLimit()) {
            return upperSetPoint;
        }
        return
                Math.abs(measurement - upperSetPoint) < Math.abs(measurement - lowerSetPoint) ?
                        upperSetPoint : lowerSetPoint;
    }
}
