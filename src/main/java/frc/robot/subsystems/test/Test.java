package frc.robot.subsystems.test;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Test extends SubsystemBase {
    private final TestIO testIO;
    private TestInputsAutoLogged inputs;

    public Test() {
        if (RobotBase.isSimulation() && !Robot.isReplay) {
            testIO = new TestReal();
        } else {
            testIO = new TestIO();
        }

        inputs = new TestInputsAutoLogged();

    }

    public void setSetpoint(double setpoint) {
        System.out.println("setpoint is: " + setpoint);
        testIO.setSetpoint(setpoint);
    }

    @Override
    public void periodic() {
        testIO.refreshAll(inputs);
        Logger.processInputs("fhahkf", inputs);
    }
}
