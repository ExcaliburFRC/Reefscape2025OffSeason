package frc.robot.superstructure.automations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.RobotStates;
import frc.robot.superstructure.Superstructure;

import java.util.HashMap;
import java.util.Map;

public class Automations {
    public Map scoreMap = new HashMap<RobotStates, Command>();
    public AllianceUtils.AlliancePose opClientPosition = new AllianceUtils.AlliancePose();
    public RobotStates scoreState = RobotStates.DEFAULT_WITH_GAME_PIECE;
    public Swerve swerve;
    public Superstructure superstructure;
    public Automations(Swerve swerve, Superstructure superstructure) {
        this.swerve = swerve;
        this.superstructure = superstructure;

        scoreMap.put(RobotStates.L1, superstructure.L1ScoreCommand());
        scoreMap.put(RobotStates.L2, superstructure.reefScoreCommand(RobotStates.L2));
        scoreMap.put(RobotStates.L3, superstructure.reefScoreCommand(RobotStates.L3));
        scoreMap.put(RobotStates.L4, superstructure.reefScoreCommand(RobotStates.L4));
        scoreMap.put(RobotStates.NET, superstructure.netScoreCommand());
    }

    public Command getNexScoreCommnd(){
        return new SequentialCommandGroup(
                swerve.driveToPoseCommand(opClientPosition.get()),
                (Command) scoreMap.get(scoreState)
        );
    }
}
