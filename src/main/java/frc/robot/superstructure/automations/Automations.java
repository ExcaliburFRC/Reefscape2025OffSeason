package frc.robot.superstructure.automations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.superstructure.RobotState;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.automations.climbMode.ClimbOperator;
import frc.robot.util.OpeningDirection;
import monologue.Logged;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.excalib.additional_utilities.AllianceUtils.FIELD_LENGTH_METERS;
import static frc.excalib.additional_utilities.AllianceUtils.FIELD_WIDTH_METERS;
import static monologue.Annotations.*;

public class Automations implements Logged {
    private Pose2d currentSetpoint = new Pose2d();
    public Swerve swerve;
//    public Superstructure superstructure;

    public ClimberSubsystem climber;
    public ClimbOperator climbOperator;

    public Automations(Swerve swerve) {
        this.swerve = swerve;
//        this.superstructure = superstructure;

        climbOperator = new ClimbOperator();
        climber = new ClimberSubsystem();
    }

    public Command alignToSide(boolean rightBranch) {
        return swerve
                .pidToPoseCommand(
                        () -> getAlignmentTargetPose(
                                rightBranch))
                .unless
                        (isInTranslationTolerance(
                                AllianceUtils.getReefCenter(),
                                () -> Constants.MAX_AUTO_ALIGNMENT_DISTANCE
                        ));
    }

    public Pose2d getAlignmentTargetPose(boolean rightBranch) {
        if (rightBranch)
            currentSetpoint = AllianceUtils.switchAlliance(getSlice().rightBranchLeftScorePose.get());
        else
            currentSetpoint = AllianceUtils.switchAlliance(getSlice().leftBranchLeftScorePose.get());
        return currentSetpoint;
    }

    @Log.NT
    public Side getSlice() {
        double angle = getAngleDiff();
        if (angle < 30 && angle > -30) {
            return Side.SOUTH;
        }
        if (angle < -30 && angle > -90) {
            return Side.SOUTH_WEST;
        }
        if (angle < -90 && angle > -150) {
            return Side.NORTH_WEST;
        }
        if (angle > 150 || angle < -150) {
            return Side.NORTH;
        }
        if (angle > 90 && angle < 150) {
            return Side.NORTH_EAST;
        }
        return Side.SOUTH_EAST;
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

    @Log.NT
    public double getAngleDiff() {
        Translation2d robotTranslation = swerve.getPose2D().getTranslation();
        if (AllianceUtils.isBlueAlliance()) {
            robotTranslation = new Translation2d(FIELD_LENGTH_METERS - robotTranslation.getX(), FIELD_WIDTH_METERS - robotTranslation.getY());
        }
        robotTranslation = robotTranslation.minus(AllianceUtils.getReefCenter());
        return robotTranslation.getAngle().getDegrees();
    }

    @Log.NT
    public Pose2d getSetpointPerSlice() {
        return AllianceUtils.switchAlliance(getSlice().leftBranchLeftScorePose.get());
    }

    @Log.NT
    public Translation2d getReefCenter() {
        return AllianceUtils.getReefCenter();
    }

    @Log.NT
    public Pose2d getCurrentSetpoint() {
        return currentSetpoint;
    }
}



