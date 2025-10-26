package frc.excalib.mechanisms.linear_extension;

import java.util.function.DoubleSupplier;

public class LinearExtentionIO {

    protected static class LinearExtentionInput {
        public DoubleSupplier positionSuppier = () -> 0;
        public DoubleSupplier angleSuppier = () -> 0;
    }
}
