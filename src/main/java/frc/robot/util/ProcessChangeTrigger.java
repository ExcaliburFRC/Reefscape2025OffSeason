package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.superstructure.Superstructure;
import monologue.Annotations;
import monologue.Logged;

import java.util.function.Supplier;

public class ProcessChangeTrigger implements Logged {
    private static Superstructure.Process lastValue = Superstructure.Process.DEFAULT;
    private Trigger trigger;
    boolean change;

    public ProcessChangeTrigger(Supplier<Superstructure.Process> supplier) {
        trigger = new Trigger(
                () -> {
                    Superstructure.Process currentValue = supplier.get();
                    change = currentValue == lastValue && lastValue != null;
                    if (change) lastValue = currentValue;
                    return change;
                }
        );
    }

    public Trigger getTrigger() {
        return trigger;
    }

    @Annotations.Log.NT
    public boolean getChange() {
        return change;
    }

    @Annotations.Log.NT
    public String getLastValue() {
        return lastValue.name();
    }

}
