package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

public class LevelChangeTrigger extends Trigger {
    private static CoralScoreState lastValue;

    public LevelChangeTrigger(Supplier<CoralScoreState> supplier) {
        super(() -> {
            CoralScoreState currentValue = supplier.get();
            boolean change = lastValue != null && currentValue != lastValue;
            if (change) lastValue = currentValue;
            return change;

        });

    }
}
