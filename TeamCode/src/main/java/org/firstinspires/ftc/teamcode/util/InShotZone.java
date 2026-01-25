package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class InShotZone {

    private Pose currentPose;

    // Top triangle shot zone
    private static final double[][] TOP_TRIANGLE = {
            {19.8, 117.2},
            {72.0,  67.2},
            {124.2, 117.2},
            {124.2, 144.0},
            {19.8,  144.0}
    };

    // Bottom triangle shot zone
    private static final double[][] BOTTOM_TRIANGLE = {
            {50.1, 11.1},
            {72.0, 28.1},
            {93.9, 11.1},
            {0.0,  96.0},
            {0.0,  48.0}
    };

    public InShotZone() {
        currentPose = new Pose();
    }

    public static boolean checkInShotZone( Pose p)
    {
        double x = p.getX();
        double y = p.getY();
        return pointInPolygon(x, y, TOP_TRIANGLE) || pointInPolygon(x, y, BOTTOM_TRIANGLE);
    }

    public void setCurrentPose(Pose pose) {
        currentPose = pose;
    }

    // Returns true if robot center is inside either shot triangle
    public boolean isInShotZone() {
        double x = currentPose.getX();
        double y = currentPose.getY();

        return pointInPolygon(x, y, TOP_TRIANGLE) ||
                pointInPolygon(x, y, BOTTOM_TRIANGLE);
    }

    // Ray-casting / even-odd point-in-polygon test
    private static boolean pointInPolygon(double x, double y, double[][] poly) {
        boolean inside = false;
        int n = poly.length;

        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = poly[i][0], yi = poly[i][1];
            double xj = poly[j][0], yj = poly[j][1];

            boolean crossesY = (yi > y) != (yj > y);
            if (!crossesY) continue;

            double xIntersect = (xj - xi) * (y - yi) / (yj - yi) + xi;
            if (x < xIntersect) inside = !inside;
        }
        return inside;
    }
}