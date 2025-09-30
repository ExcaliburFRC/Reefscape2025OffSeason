package frc.robot.superstructure.automations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.superstructure.RobotState;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.automations.climbMode.ClimbOperator;
import frc.robot.util.OpeningDirection;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Automations {
    public Map<RobotState, Command> scoreMap = new HashMap<>();
    public RobotState scoreState = RobotState.DEFAULT_WITH_GAME_PIECE;

    public Swerve swerve;
    public Superstructure superstructure;

    public ClimberSubsystem climber;
    public ClimbOperator climbOperator;

    public Automations(Swerve swerve, Superstructure superstructure) {
        this.swerve = swerve;
        this.superstructure = superstructure;

        climbOperator = new ClimbOperator();
        climber = new ClimberSubsystem();
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
        FieldConstants.Side currentSide = getClosestSide(currentPose);
        if (rightBranch)
            return currentSide.rightBrachPose;
        return currentSide.leftBranchPose;
    }


    private FieldConstants.Side getClosestSide(Pose2d currentPose) {
        Translation2d diff = currentPose.minus(swerve.getPose2D()).getTranslation();
        double robotAngleByReefCenter = diff.getAngle().getDegrees();
        if (robotAngleByReefCenter < 30 && robotAngleByReefCenter > -30) {
            return FieldConstants.Side.NORTH;
        }
        if (robotAngleByReefCenter < -30 && robotAngleByReefCenter > -90) {
            return FieldConstants.Side.NORTH_EAST;
        }
        if (robotAngleByReefCenter < -90 && robotAngleByReefCenter > -150) {
            return FieldConstants.Side.NORTH_WEST;
        }
        if (robotAngleByReefCenter > 150 || robotAngleByReefCenter < -150) {
            return FieldConstants.Side.SOUTH;
        }
        if (robotAngleByReefCenter > 90 && robotAngleByReefCenter < 150) {
            return FieldConstants.Side.SOUTH_EAST;
        }
        return FieldConstants.Side.SOUTH_WEST;
    }

    public BooleanSupplier isInTranslationTolerance(Translation2d translationCenter, DoubleSupplier tolerance) {
        Translation2d cuurentPose = swerve.getPose2D().getTranslation();
        return () -> cuurentPose.getDistance(translationCenter) < tolerance.getAsDouble();
    }

    public OpeningDirection getOpeningDirection() {
        return OpeningDirection.LEFT;
    }

    public double getDeltaPostions(Translation2d poseA, Translation2d poseB) {
        return Math.sqrt(
                Math.pow(poseA.getX() - poseB.getX(), 2)
                        + Math.pow(poseA.getY() - poseB.getY(), 2)
        );
    }

}



