package frc.robot.superstructure;

public enum FieldPoses {
    BRANCH_1(0, 0),
    BRANCH_2(0, 0),
    BRANCH_3(0, 0),
    BRANCH_4(0, 0),
    BRANCH_5(0, 0),
    BRANCH_6(0, 0),
    BRANCH_7(0, 0),
    BRANCH_8(0, 0),
    BRANCH_9(0, 0),
    BRANCH_10(0, 0),
    BRANCH_11(0, 0),
    BRANCH_12(0, 0),


    CHAIN_1(0, 0),
    CHAIN_2(0, 0),
    CHAIN_3(0, 0),


    PROCESSOR(0, 0),

    NET_1(0,0),
    NET_2(0,0),
    NET_3(0,0),
    NET_4(0,0),
    NET_5(0,0),
    NET_6(0,0);






    final int x;
    final int y;
    private FieldPoses(int x, int y){
        this.x = x;
        this.y = y;
    }
}
