package frc.excalib.additional_utilities;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class PS5Controller extends CommandPS5Controller {
    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */

    private boolean isPrimary;
    private final XboxController simulatedController;

    public PS5Controller(int port, boolean isPrimary, int simulationControllerPort) {
        super(port);
        this.isPrimary = isPrimary;
        simulatedController = new XboxController(simulationControllerPort);
    }

    public double getJoystickDistance(Position joystickPosition) {
        if (joystickPosition == Position.LEFT) {
            return Math.sqrt(Math.pow(super.getLeftX(), 2) - Math.pow(super.getLeftY(), 2));
        } else {
            return Math.sqrt(Math.pow(super.getRightX(), 2) - Math.pow(super.getRightY(), 2));
        }
    }

    public Command vibrateControllerCommand(double secondsDuration, double intesity, GenericHID.RumbleType rumbleType) {
        return Commands.runEnd(
                () -> simulatedController.setRumble(rumbleType, intesity),
                () -> simulatedController.setRumble(rumbleType, 0)
        ).withTimeout(secondsDuration).ignoringDisable(true);
    }


    public boolean isPrimary() {
        return isPrimary;
    }

    public void setPrimary(boolean primary) {
        isPrimary = primary;
    }
}
