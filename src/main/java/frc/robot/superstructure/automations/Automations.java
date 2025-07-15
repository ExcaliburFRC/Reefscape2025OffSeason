package frc.robot.superstructure.automations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.superstructure.RobotStates;
import frc.robot.superstructure.Superstructure;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

    public Command alignToSide(boolean rightBranch) {
        return swerve
                .pidToPoseCommand(
                        () -> getAlignmentTargetPose(
                                swerve.getPose2D(),
                                rightBranch)
                                .get())
                .unless
                        (isInTranslationTolerance(
                                AllianceUtils.getReefCenter(),
                                () -> Constants.MAX_AUTO_ALIGNMENT_DISTANCE
                        ));
    }

    private AllianceUtils.AlliancePose getAlignmentTargetPose(Pose2d currentPose, boolean rightBranch) {
        Constants.FieldConstants.Side currentSide = getClosestSide(currentPose);

        if (rightBranch)
            return currentSide.rightBrachPose;
        return currentSide.leftBranchPose;
    }


    private Constants.FieldConstants.Side getClosestSide(Pose2d currentPose) {
        Translation2d diff = currentPose.minus(swerve.getPose2D()).getTranslation();
        double robotAngleByReefCenter = diff.getAngle().getDegrees();
        if (robotAngleByReefCenter < 30 && robotAngleByReefCenter > -30) {
            return Constants.FieldConstants.Side.NORTH;
        }
        if (robotAngleByReefCenter < -30 && robotAngleByReefCenter > -90) {
            return Constants.FieldConstants.Side.NORTH_EAST;
        }
        if (robotAngleByReefCenter < -90 && robotAngleByReefCenter > -150) {
            return Constants.FieldConstants.Side.NORTH_WEST;
        }
        if (robotAngleByReefCenter > 150 || robotAngleByReefCenter < -150) {
            return Constants.FieldConstants.Side.SOUTH;
        }
        if (robotAngleByReefCenter > 90 && robotAngleByReefCenter < 150) {
            return Constants.FieldConstants.Side.SOUTH_EAST;
        }
        return Constants.FieldConstants.Side.SOUTH_WEST;
    }

    public BooleanSupplier isInTranslationTolerance(Translation2d translationCenter, DoubleSupplier tolerance) {
        Translation2d cuurentPose = swerve.getPose2D().getTranslation();
        return () -> cuurentPose.getDistance(translationCenter) < tolerance.getAsDouble();
    }
}



