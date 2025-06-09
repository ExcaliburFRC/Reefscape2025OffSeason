package frc.robot.superstructure;

import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;

import java.util.function.BooleanSupplier;

public class Superstructure {
    private final ArmSubsystem armSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final Intake intakeSubsystem;
    private final Gripper gripperSubsystem;

    public Superstructure() {
        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake(IntakeState.STOW);
        gripperSubsystem = new Gripper();
        elevatorSubsystem.setArmAngle(armSubsystem.getAngleSupplier());
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem.getElevatorHeight());
        armSubsystem.setIntakeOpen(intakeSubsystem.isIntakeOpen());

    }
}
