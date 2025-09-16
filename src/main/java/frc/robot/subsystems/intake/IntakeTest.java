package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;

public class IntakeTest {
    TalonFXMotor centralizerMotor;
    Mechanism centralizer;
    Mechanism roller;
    TalonFXMotor rollerMotor;
    public IntakeTest() {
         centralizerMotor = new TalonFXMotor(32);
         centralizer = new Mechanism(centralizerMotor);
         rollerMotor = new TalonFXMotor(31);
         roller = new Mechanism(rollerMotor);
    }

    public Command test() {
        return new ParallelCommandGroup(
                centralizer.manualCommand(() -> 0.1),
                roller.manualCommand(() -> 0.1)
        );
    }
}
