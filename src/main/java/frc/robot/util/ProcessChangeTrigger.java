package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.superstructure.Superstructure;

import java.util.function.Supplier;

public class ProcessChangeTrigger extends Trigger {
    private static Superstructure.Process lastValue;

    public ProcessChangeTrigger(Supplier<Superstructure.Process> supplier) {
        super(() -> {
            Superstructure.Process currentValue = supplier.get();
            boolean change = lastValue != null && currentValue != lastValue;
            if (change) lastValue = currentValue;
            return change;

        });

    }

}
