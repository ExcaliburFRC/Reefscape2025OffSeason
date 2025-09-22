package frc.robot.superstructure.automations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.superstructure.RobotStates;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.automations.climbMode.ClimbOperator;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Automations {
    public Map<RobotStates, Command> scoreMap = new HashMap<>();
    public RobotStates scoreState = RobotStates.DEFAULT_WITH_GAME_PIECE;

    public Swerve swerve;
    public Superstructure superstructure;

    public ClimberSubsystem climber;
    public ClimbOperator climbOperator;

    public Automations(Swerve swerve, Superstructure superstructure) {
        this.swerve = swerve;
        this.superstructure = superstructure;

//        scoreMap.put(RobotStates.SCORE_L1, superstructure.L1ScoreCommand());
//        scoreMap.put(RobotStates.L2, superstructure.openToScoreCommand(RobotStates.L2));
//        scoreMap.put(RobotStates.L3, superstructure.openToScoreCommand(RobotStates.L3));
//        scoreMap.put(RobotStates.L4, superstructure.openToScoreCommand(RobotStates.L4));
//        scoreMap.put(RobotStates.NET, superstructure.netScoreCommand());

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
        double leftBeanXFactor = swerve.getPose2D().getX() * Math.cos(swerve.getPose2D().getRotation().getRadians());
        double leftBeanYFactor = swerve.getPose2D().getY() * Math.sin(swerve.getPose2D().getRotation().getRadians());
        AllianceUtils.AlliancePose leftBeamFromRobot = new AllianceUtils.AlliancePose(
                new Translation2d(
                        swerve.getPose2D().getX() + leftBeanXFactor,
                        swerve.getPose2D().getY() + leftBeanYFactor),
                swerve.getPose2D().getRotation()
        );

        double rightBeanXFactor = swerve.getPose2D().getX() * Math.cos(swerve.getPose2D().getRotation().getRadians());
        double rightBeanYFactor = swerve.getPose2D().getY() * Math.sin(swerve.getPose2D().getRotation().getRadians());
        AllianceUtils.AlliancePose rightBeamFromRobot = new AllianceUtils.AlliancePose(
                new Translation2d(
                        swerve.getPose2D().getX() + leftBeanXFactor - Math.PI,
                        swerve.getPose2D().getY() + leftBeanYFactor - Math.PI),
                swerve.getPose2D().getRotation()
        );

        if (
                getDeltaPostions(rightBeamFromRobot.get().getTranslation(), AllianceUtils.getReefCenter())>
                getDeltaPostions(leftBeamFromRobot.get().getTranslation(), AllianceUtils.getReefCenter())) {
            return OpeningDirection.LEFT;
        }

        return OpeningDirection.RIGHT;

    }

    public enum OpeningDirection {
        LEFT,
        RIGHT
    }

    public double getDeltaPostions(Translation2d poseA, Translation2d poseB) {
        return Math.sqrt(
                Math.pow(poseA.getX() - poseB.getX(), 2)
                        + Math.pow(poseA.getY() - poseB.getY(), 2)
        );
    }

}



