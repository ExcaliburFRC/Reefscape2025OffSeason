package frc.robot.superstructure.automations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.superstructure.automations.climbMode.ClimbOperator;
import frc.robot.util.OpeningDirection;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.excalib.additional_utilities.AllianceUtils.*;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH;
import static monologue.Annotations.*;

public class Automations implements Logged {
    private Pose2d currentSetpoint = new Pose2d();
    public Swerve swerve;

    public ClimbOperator climbOperator;

    public Automations(Swerve swerve) {
        this.swerve = swerve;
        climbOperator = new ClimbOperator();
    }

    public Command alignToSide(boolean rightBranch) {
        return swerve
                .pidToPoseCommand(
                        () -> getAlignmentTargetPose(
                                rightBranch)
                );
//                .unless(
//                        isInTranslationTolerance(
//                                AllianceUtils.getReefCenter(),
//                                () -> Constants.MAX_AUTO_ALIGNMENT_DISTANCE
//                        ));
    }

    @Log.NT
    public boolean isLeftReefScore() {
        double toCheck = getRotationCheck();
        boolean flag;
        if (toCheck > 0) {
            flag =  true;
        } else {
            flag = false;
        }
        if (isRedAlliance()){
            return !flag;
        }
        return flag;
    }

    public Pose2d getAlignmentTargetPose(boolean rightBranch) {
        OpeningDirection openingDirection = isLeftReefScore() ? OpeningDirection.LEFT : OpeningDirection.RIGHT;

        Translation2d targetTranslation = new Translation2d();
        if (!rightBranch && openingDirection.equals(OpeningDirection.LEFT)) {
            targetTranslation = Constants.FieldConstants.B1_LEFT_SCORE;
        } else if (!rightBranch && openingDirection.equals(OpeningDirection.RIGHT)) {

            targetTranslation = Constants.FieldConstants.B1_RIGHT_SCORE;
        } else if (openingDirection.equals(OpeningDirection.LEFT)) {
            targetTranslation = Constants.FieldConstants.B12_LEFT_SCORE;
        } else {
            targetTranslation = Constants.FieldConstants.B12_RIGHT_SCORE;
        }

        Pose2d pose2d = getSlice().getTargetPose(targetTranslation, openingDirection).get();
        Translation2d translation2d = pose2d.getTranslation().plus(getReefCenter()).minus(new Translation2d(Units.inchesToMeters(176.746),
                FIELD_WIDTH / 2));

        if (isRedAlliance()) {
//            return new Pose2d(translation2d, pose2d.getRotation().plus(Rotation2d.k180deg));
            return new Pose2d(translation2d, pose2d.getRotation());
        }
        return pose2d;
    }

    @Log.NT
    public boolean atL2Slice() {
        Side slice = getSlice();
        if (slice.equals(Side.SOUTH) || slice.equals(Side.NORTH_WEST) || slice.equals(Side.NORTH_EAST)) {
            return false;
        }
        return true;
    }

    @Log.NT
    public Side getSlice() {
        double angle = getAngleDiff();
        if (angle < 30 && angle > -30) {
            return Side.NORTH;
        }
        if (angle < -30 && angle > -90) {
            return Side.NORTH_EAST;
        }
        if (angle < -90 && angle > -150) {
            return Side.SOUTH_EAST;
        }
        if (angle > 150 || angle < -150) {
            return Side.SOUTH;
        }
        if (angle > 90 && angle < 150) {
            return Side.SOUTH_WEST;
        }
        return Side.NORTH_WEST;
    }

    public BooleanSupplier isInTranslationTolerance(Translation2d translationCenter, DoubleSupplier tolerance) {
        Translation2d current = swerve.getPose2D().getTranslation();
        return () -> current.getDistance(translationCenter) < tolerance.getAsDouble();
    }

    @Log.NT
    public Trigger openToRightTrigger() {
        return new Trigger(() -> getSlice().angle.getDegrees() - swerve.getPose2D().getRotation().getDegrees() < 90);
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
        robotTranslation = robotTranslation.minus(AllianceUtils.getReefCenter());
        if (AllianceUtils.isRedAlliance()) {
            return robotTranslation.getAngle().plus(Rotation2d.k180deg).getDegrees();
        }
        return robotTranslation.getAngle().getDegrees();
    }

    @Log.NT
    public Pose2d getRightSetpointPerSlice() {
        return getAlignmentTargetPose(true);
    }

    @Log.NT
    public Pose2d getLeftSetpointPerSlice() {
        return getAlignmentTargetPose(false);
    }

    @Log.NT
    public Translation2d getReefCenter() {
        return AllianceUtils.getReefCenter();
    }

    @Log.NT
    public Pose2d getCurrentSetpoint() {
        return currentSetpoint;
    }

    @Log.NT
    public double getRotationCheck() {
        Rotation2d val = swerve.getRotation2D().minus(getSlice().angle);
//        if (isRedAlliance()) {
            val.plus(Rotation2d.k180deg);
//        }
        return val.getDegrees();
    }
}



