package org.firstinspires.ftc.teamcode.util;

public class DecideOrder {
    //this will return the order in which to shoot
    /*
    motif:
    Blue Goal: 20
    Motif GPP: 21
    Motif PGP: 22
    Motif PPG: 23
    Red Goal: 24

    positionGreen:
    1 = left
    2 = right
    3 = back

    return:
    1: LL
    2: RR
    3: LR
    4: RL
     */
    public int order(int motif, int positionGreen) {
        if (motif == 21) {
            if (positionGreen == 1) {
                return 3;
            } else  {
                return 4;
            }
        } else if (motif == 22) {
            if (positionGreen == 1) {
                return 4;
            } else if (positionGreen == 3) {
                return 2;
            } else {
                return 3;
            }
        } else {
            if (positionGreen == 1) {
                return 2;
            } else if (positionGreen == 2) {
                return 1;
            } else {
                return 3;
            }
        }
    }
}
