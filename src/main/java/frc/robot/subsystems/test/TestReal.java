package frc.robot.subsystems.test;

public class TestReal extends TestIO {
    double height = 0;
    double setpoint = 0;

    public TestReal() {

    }

    @Override
    protected void refreshAll(TestInputsAutoLogged inputs) {
        inputs.height = this.height;
        inputs.setpoint = this.setpoint;
    }

    @Override
    public void setSetpoint(double setpoint) {
        System.out.println("fshjkdfhsf567345");
        this.setpoint = setpoint;
    }
}
