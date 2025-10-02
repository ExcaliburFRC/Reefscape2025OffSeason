package frc.excalib.additional_utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import jdk.jshell.execution.LoaderDelegate;
import monologue.Annotations;
import monologue.Logged;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.FieldConstants.*;

/**
 * the purpose of this class is too supply different utility functions for functionality
 * that depends on the robot alliance.
 * @author Shai Grossman
 */
public class AllianceUtils {
    public static final double FIELD_LENGTH_METERS = 17.548;
    public static final double FIELD_WIDTH_METERS = 8.052;

    /**
     * @return whether the robot is on the blue alliance
     */
    public static boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) return alliance.get().equals(Blue);
        DriverStation.reportError("DS alliance empty!", false);
        return true;
    }

    public static boolean isRedAlliance() {
        return !isBlueAlliance();
    }

    public static int getdirection() {
        if (isBlueAlliance())
            return 1;
        return -1;
    }

    public static Translation2d getReefCenter(){
        return CURRENT_REEF_CENTER;
    }

    /**
     * Converts a pose to the pose relative to the current driver station alliance.
     *
     * @param bluePose the current blue alliance pose
     * @return the converted pose
     */
    public static Pose2d toAlliancePose(Pose2d bluePose) {
        if (isBlueAlliance()) return bluePose;
        return switchAlliance(bluePose);
    }

    public static Pose2d switchAlliance(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(), FIELD_WIDTH_METERS - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromDegrees(180))
        );
    }

    public static Pose2d mirrorAlliance(Pose2d pose) {
        return new Pose2d(
                FIELD_LENGTH_METERS - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation())
        );
    }

    public static class AlliancePose {
        private Pose2d pose;

        public AlliancePose(double x, double y, double degrees) {
            this.pose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
        }

        public AlliancePose() {
            this.pose = new Pose2d(new Translation2d(), new Rotation2d());
        }

        public AlliancePose(Translation2d translation, Rotation2d rotation) {
            this.pose = new Pose2d(translation, rotation);
        }

        public AlliancePose(double degrees) {
            this.pose = new Pose2d(0, 0, Rotation2d.fromDegrees(degrees));
        }

        public Pose2d get() {
            return toAlliancePose(pose);
        }
    }
}
