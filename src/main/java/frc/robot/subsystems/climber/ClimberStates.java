package frc.robot.subsystems.climber;

public enum ClimberStates {
    STOW(90),
    OPEN(45),
    RETRACT(135);



    private final double angle;
    private ClimberStates(double height){
        this.angle = height;
    }

    public double getHeight() {
        return angle;
    }
}
