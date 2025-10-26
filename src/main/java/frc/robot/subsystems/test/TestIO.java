package frc.robot.subsystems.test;


import org.littletonrobotics.junction.AutoLog;

public class TestIO {

    @AutoLog
    protected static class TestInputs {
        double setpoint = 0;
        double height = 2;
    }

    protected void setSetpoint(double setpoint) {}

    protected void refreshAll(TestInputsAutoLogged inputs) {
    }

    public void manualCommand(double v) {
    }

}
