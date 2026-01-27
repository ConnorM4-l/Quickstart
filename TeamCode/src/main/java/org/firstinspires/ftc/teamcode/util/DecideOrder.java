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
        if (positionGreen < 1 || positionGreen > 3) return -1;

        switch (motif) {
            case 21: // GPP: green must be first
                if (positionGreen == 1) return 3; // LR: shoot L first (green), then R (purple)
                if (positionGreen == 2) return 4; // RL: shoot R first (green), then L (purple)
                return -1; // green in back can't be shot first with LL/RR/LR/RL patterns

            case 22: // PGP: purple then green
                if (positionGreen == 1) return 4; // RL: shoot R (purple), then L (green)
                if (positionGreen == 2) return 3; // LR: shoot L (purple), then R (green)
                // green back: first shot can be either purple (L or R),
                // then you want green to be second (depends on your feeder behavior).
                // Keep your old choice:
                return 2; // RR (you can swap to 1 if your robot feeds green to left after first shot)

            case 23: // PPG: two purples first
                if (positionGreen == 1) return 2; // RR: purples are R + back -> clear purples first
                if (positionGreen == 2) return 1; // LL
                return 3; // green back: both L and R are purple, so LR (or RL) works

            default:
                return -1; // should never happen since limelight only outputs 21/22/23
        }
    }
}
