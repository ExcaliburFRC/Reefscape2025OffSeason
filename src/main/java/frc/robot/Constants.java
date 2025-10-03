// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.imu.Pigeon;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.swerve.ModulesHolder;
import frc.excalib.swerve.Swerve;
import frc.excalib.swerve.SwerveModule;

public final class Constants {
    public static class SuperstructureConstants {
        public static final double HANDOFF_TIME_DELAY = 0.2;
    }

    public static class SwerveConstants {
        public static final int FRONT_LEFT_DRIVE_ID = 20;
        public static final int FRONT_RIGHT_DRIVE_ID = 10;
        public static final int BACK_RIGHT_DRIVE_ID = 40;
        public static final int BACK_LEFT_DRIVE_ID = 30;

        public static final int FRONT_LEFT_ROTATION_ID = 22;
        public static final int FRONT_RIGHT_ROTATION_ID = 12;
        public static final int BACK_RIGHT_ROTATION_ID = 42;
        public static final int BACK_LEFT_ROTATION_ID = 32;

        public static final int GYRO_ID = 3;
        public static final String SWERVE_CANBUS = "CTRESwerve";

        private static final double PID_TOLERANCE = 0.01;

        public static final double TRACK_WIDTH = 0.726; // m
        public static final Translation2d FRONT_LEFT_TRANSLATION =
                new Translation2d(
                        TRACK_WIDTH / 2, TRACK_WIDTH / 2
                );
        public static final Translation2d FRONT_RIGHT_TRANSLATION =
                new Translation2d(
                        TRACK_WIDTH / 2, -TRACK_WIDTH / 2
                );
        public static final Translation2d BACK_LEFT_TRANSLATION =
                new Translation2d(
                        -TRACK_WIDTH / 2, TRACK_WIDTH / 2
                );
        public static final Translation2d BACK_RIGHT_TRANSLATION =
                new Translation2d(
                        -TRACK_WIDTH / 2, -TRACK_WIDTH / 2
                );

        public static final double MAX_MODULE_VEL = 4.45;
        public static final double MAX_FRONT_ACC = 2;
        public static final double MAX_SIDE_ACC = 6;
        public static final double MAX_SKID_ACC = 9;
        public static final double MAX_FORWARD_ACC = 9;
        public static final double MAX_VEL = 4;
        public static final double MAX_OMEGA_RAD_PER_SEC = 4;
        public static final double MAX_OMEGA_RAD_PER_SEC_SQUARE = 4;

        public static final PathConstraints MAX_PATH_CONSTRAINTS = new PathConstraints(
                MAX_VEL,
                MAX_SKID_ACC,
                MAX_OMEGA_RAD_PER_SEC,
                MAX_OMEGA_RAD_PER_SEC_SQUARE,
                12.0,
                false
        );

        public static final CANcoder FRONT_RIGHT_ABS_ENCODER = new CANcoder(11, SWERVE_CANBUS);
        private static final CANcoder FRONT_LEFT_ABS_ENCODER = new CANcoder(21, SWERVE_CANBUS);
        private static final CANcoder BACK_LEFT_ABS_ENCODER = new CANcoder(31, SWERVE_CANBUS);
        private static final CANcoder BACK_RIGHT_ABS_ENCODER = new CANcoder(41, SWERVE_CANBUS);

        private static final double VELOCITY_CONVERSION_FACTOR = Units.inchesToMeters(4) * Math.PI / 6.12;
        private static final double POSITION_CONVERSION_FACTOR = Units.inchesToMeters(4) * Math.PI / 6.12;
        private static final double ROTATION_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI) / (21.4285714);

        public static final PIDConstants TRANSLATION_PID_PP_CONSTANTS = new PIDConstants(10.0, 0.0, 0.0); //TODO
        public static final PIDConstants ANGLE_PID_PP_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
        public static final Gains ANGLE_PID_GAINS = new Gains(0.1, 0, 0);
        public static final Gains TRANSLATION_PID_GAINS = new Gains(3.62, 0, 0);

        private static final IMU GYRO = new Pigeon(GYRO_ID, SWERVE_CANBUS, new Rotation3d());

        public static Swerve configureSwerve(Pose2d initialPose) {
            return new Swerve(
                    new ModulesHolder(
                            new SwerveModule(
                                    new TalonFXMotor(FRONT_LEFT_DRIVE_ID, SWERVE_CANBUS),
                                    new TalonFXMotor(FRONT_LEFT_ROTATION_ID, SWERVE_CANBUS),
                                    new Gains(7.5, 0, 0, 0.192, 0, 0, 0),
                                    new Gains(0, 0, 0, 0.17418 * 0.5, 2.254, 0, 0),
                                    PID_TOLERANCE,
                                    FRONT_LEFT_TRANSLATION,
                                    () -> FRONT_LEFT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            ),
                            new SwerveModule(
                                    new TalonFXMotor(FRONT_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                                    new TalonFXMotor(FRONT_RIGHT_ROTATION_ID, SWERVE_CANBUS),
                                    new Gains(7.5, 0, 0, 0.192, 0, 0, 0),
                                    new Gains(0, 0, 0, 0.17418 * 0.5, 2.19, 0, 0),
                                    PID_TOLERANCE,
                                    FRONT_RIGHT_TRANSLATION,
                                    () -> FRONT_RIGHT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            ),
                            new SwerveModule(
                                    new TalonFXMotor(BACK_LEFT_DRIVE_ID, SWERVE_CANBUS),
                                    new TalonFXMotor(BACK_LEFT_ROTATION_ID, SWERVE_CANBUS),
                                    new Gains(7.5, 0, 0, 0.192, 0, 0, 0),
                                    new Gains(0, 0, 0, 0.17418 * 0.5, 2.232, 0, 0),
                                    PID_TOLERANCE,
                                    BACK_LEFT_TRANSLATION,
                                    () -> BACK_LEFT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            ),
                            new SwerveModule(
                                    new TalonFXMotor(BACK_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                                    new TalonFXMotor(BACK_RIGHT_ROTATION_ID, SWERVE_CANBUS),
                                    new Gains(7.5, 0, 0, 0.192, 0, 0, 0),
                                    new Gains(0, 0, 0, 0.17418 * 0.5, 2.184, 0, 0),
                                    PID_TOLERANCE,
                                    BACK_RIGHT_TRANSLATION,
                                    () -> BACK_RIGHT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            )),
                    GYRO,
                    initialPose
            );
        }
    }

    public static class FieldConstants {
        public static double FIELD_LENGTH = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldLength();
        public static double FIELD_WIDTH = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getFieldWidth();

        public static Translation2d CURRENT_REEF_CENTER = AllianceUtils.toAlliancePose(
                new Pose2d(
                        new Translation2d(
                                Units.inchesToMeters(176.746),
                                FIELD_WIDTH / 2
                        ),
                        new Rotation2d())
        ).getTranslation();

        public static Translation2d B1_LEFT_SCORE = new Translation2d(CURRENT_REEF_CENTER.getX() - 1.276, CURRENT_REEF_CENTER.getY() + 0.45);
        public static Translation2d B1_RIGHT_SCORE = new Translation2d(CURRENT_REEF_CENTER.getX() - 1.276, CURRENT_REEF_CENTER.getY() + 0.322);
        public static Translation2d B12_LEFT_SCORE = new Translation2d(CURRENT_REEF_CENTER.getX() - 1.276, CURRENT_REEF_CENTER.getY() - 0.329);
        public static Translation2d B12_RIGHT_SCORE = new Translation2d(CURRENT_REEF_CENTER.getX() - 1.276, CURRENT_REEF_CENTER.getY() - 0.45);
        public static Translation2d BASE_ALGAE = new Translation2d();

        public static final AllianceUtils.AlliancePose[] LEFT_BRANCHES_LEFT_SCORE = {
                new AllianceUtils.AlliancePose(B1_LEFT_SCORE, Rotation2d.fromDegrees(180)),
                new AllianceUtils.AlliancePose(B1_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120)),
                new AllianceUtils.AlliancePose(B1_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(60)),
                new AllianceUtils.AlliancePose(B1_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-180)), new Rotation2d()),
                new AllianceUtils.AlliancePose(B1_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AllianceUtils.AlliancePose(B1_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120)),
        };
        public static final AllianceUtils.AlliancePose[] LEFT_BRANCHES_RIGHT_SCORE = {
                new AllianceUtils.AlliancePose(B1_RIGHT_SCORE,new Rotation2d()),
                new AllianceUtils.AlliancePose(B1_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-60)),
                new AllianceUtils.AlliancePose(B1_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(-120)),
                new AllianceUtils.AlliancePose(B1_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-180)), new Rotation2d(180)),
                new AllianceUtils.AlliancePose(B1_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(120)),
                new AllianceUtils.AlliancePose(B1_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(60)),
        };

        public static final AllianceUtils.AlliancePose[] RIGHT_BRANCHES_LEFT_SCORE = {
                new AllianceUtils.AlliancePose(B12_LEFT_SCORE, Rotation2d.fromDegrees(180)),
                new AllianceUtils.AlliancePose(B12_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120)),
                new AllianceUtils.AlliancePose(B12_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(60)),
                new AllianceUtils.AlliancePose(B12_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-180)), new Rotation2d()),
                new AllianceUtils.AlliancePose(B12_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AllianceUtils.AlliancePose(B12_LEFT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120))
        };

        public static final AllianceUtils.AlliancePose[] RIGHT_BRANCHES_RIGHT_SCORE = {
                new AllianceUtils.AlliancePose(B12_RIGHT_SCORE, new Rotation2d()),
                new AllianceUtils.AlliancePose(B12_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-60)),
                new AllianceUtils.AlliancePose(B12_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(-120)),
                new AllianceUtils.AlliancePose(B12_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-180)), new Rotation2d(180)),
                new AllianceUtils.AlliancePose(B12_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(120)),
                new AllianceUtils.AlliancePose(B12_RIGHT_SCORE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(60))
        };

        public static final AllianceUtils.AlliancePose[] ALGAE_POSES = {
                new AllianceUtils.AlliancePose(BASE_ALGAE, Rotation2d.fromDegrees(0)),
                new AllianceUtils.AlliancePose(BASE_ALGAE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-60)),
                new AllianceUtils.AlliancePose(BASE_ALGAE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(-120)),
                new AllianceUtils.AlliancePose(BASE_ALGAE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-180)), Rotation2d.fromDegrees(180)),
                new AllianceUtils.AlliancePose(BASE_ALGAE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(120)),
                new AllianceUtils.AlliancePose(BASE_ALGAE.rotateAround(CURRENT_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(60)),
        };

        public enum Side {
            NORTH(LEFT_BRANCHES_LEFT_SCORE[0], LEFT_BRANCHES_RIGHT_SCORE[0], RIGHT_BRANCHES_RIGHT_SCORE[0], RIGHT_BRANCHES_LEFT_SCORE[0], ALGAE_POSES[0], 0),
            NORTH_EAST(LEFT_BRANCHES_LEFT_SCORE[1], LEFT_BRANCHES_RIGHT_SCORE[1], RIGHT_BRANCHES_RIGHT_SCORE[1], RIGHT_BRANCHES_LEFT_SCORE[1], ALGAE_POSES[1], -60),
            SOUTH_EAST(LEFT_BRANCHES_LEFT_SCORE[2], LEFT_BRANCHES_RIGHT_SCORE[2], RIGHT_BRANCHES_RIGHT_SCORE[2], RIGHT_BRANCHES_LEFT_SCORE[2], ALGAE_POSES[2], -120),
            SOUTH(LEFT_BRANCHES_LEFT_SCORE[3], LEFT_BRANCHES_RIGHT_SCORE[3], RIGHT_BRANCHES_RIGHT_SCORE[3], RIGHT_BRANCHES_LEFT_SCORE[3], ALGAE_POSES[3], 180),
            SOUTH_WEST(LEFT_BRANCHES_LEFT_SCORE[4], LEFT_BRANCHES_RIGHT_SCORE[4], RIGHT_BRANCHES_RIGHT_SCORE[4], RIGHT_BRANCHES_LEFT_SCORE[4], ALGAE_POSES[4], 60),
            NORTH_WEST(LEFT_BRANCHES_LEFT_SCORE[5], LEFT_BRANCHES_RIGHT_SCORE[5], RIGHT_BRANCHES_RIGHT_SCORE[5], RIGHT_BRANCHES_LEFT_SCORE[5], ALGAE_POSES[5],120);

            public final AllianceUtils.AlliancePose leftBranchLeftScorePose,
                    leftBranchRightScorePose, rightBranchRightScorePose,
                    rightBranchLeftScorePose, alagePose;
            public final double angle;

            Side(AllianceUtils.AlliancePose leftBranchLeftScorePose,
                 AllianceUtils.AlliancePose leftBranchRightScorePose,
                 AllianceUtils.AlliancePose rightBranchRightScorePose,
                 AllianceUtils.AlliancePose rightBranchLeftScorePose,
                 AllianceUtils.AlliancePose alagePose,
                 double angle) {
                this.leftBranchRightScorePose = leftBranchRightScorePose;
                this.leftBranchLeftScorePose = leftBranchLeftScorePose;
                this.rightBranchRightScorePose = rightBranchRightScorePose;
                this.rightBranchLeftScorePose = rightBranchLeftScorePose;
                this.alagePose = alagePose;
                this.angle = angle;
            }
        }

    }

    public static int DRIVER_CONTROLLER_PORT = 0;
    public static double MAX_AUTO_ALIGNMENT_DISTANCE = 0;

    public static final int AURORA_CLIENT_PORT = 5000;


}
