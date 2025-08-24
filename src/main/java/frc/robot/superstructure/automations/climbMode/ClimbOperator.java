package frc.robot.superstructure.automations.climbMode;

import edu.wpi.first.math.geometry.Pose2d;
import frc.excalib.additional_utilities.AllianceUtils;

public class ClimbOperator {

    private final ClimbNode left = new ClimbNode(new AllianceUtils.AlliancePose(), new AllianceUtils.AlliancePose());
    private final ClimbNode center = new ClimbNode(new AllianceUtils.AlliancePose(), new AllianceUtils.AlliancePose());
    private final ClimbNode right = new ClimbNode(new AllianceUtils.AlliancePose(), new AllianceUtils.AlliancePose());

    ClimbNode current = left;

    public ClimbOperator(){
        left.setNext(center);
        center.setNext(right);
        right.setNext(left);
        right.setPrev(center);
        center.setPrev(left);
        left.setPrev(right);
    }

    public void goToNext(){
        current = current.getNext();
    }

    public void goToPrev(){
        current = current.getPrev();
    }

    public Pose2d getPrePose(){
        return current.prePose.get();
    }

    public Pose2d getPose(){
        return current.pose.get();
    }

}
