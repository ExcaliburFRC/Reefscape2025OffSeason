package frc.robot.superstructure.automations.climbMode;

import frc.excalib.additional_utilities.AllianceUtils;

public class ClimbNode {
    ClimbNode next, prev;
    AllianceUtils.AlliancePose prePose, pose;

    ClimbNode(AllianceUtils.AlliancePose prePose, AllianceUtils.AlliancePose pose){
        this.pose = pose;
        this.prePose = prePose;
    }
    public AllianceUtils.AlliancePose getPrePose() {
        return prePose;
    }

    public void setPrePose(AllianceUtils.AlliancePose prePose) {
        this.prePose = prePose;
    }

    public AllianceUtils.AlliancePose getPose() {
        return pose;
    }

    public void setPose(AllianceUtils.AlliancePose pose) {
        this.pose = pose;
    }

    public ClimbNode getNext() {
        return next;
    }

    public ClimbNode getPrev() {
        return next;
    }

    public void setNext(ClimbNode next){
        this.next = next;
    }

    public void setPrev(ClimbNode prev){
        this.prev = prev;
    }
}
