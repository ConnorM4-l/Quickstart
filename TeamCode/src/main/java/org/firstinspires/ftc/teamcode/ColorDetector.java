package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
//wiring & config
//On the RC (phone/Control Hub), add your Logitech as “Webcam 1” in the configuration.
//
//        Resolution/orientation can be tuned in startStreaming(...).
//
//If your lighting is tricky, tune the HSV scalars in the pipeline (raise V/S minimums to knock out shadows; adjust hue spans to isolate target game elements).
//
//quick tuning tips (GM0-style)
//Use the RC’s monitor view (or FTC Dashboard if you prefer) to see the overlays.
//
//Start with a high S and V minimum (e.g., 120–150) to reject gray background.
//
//Red wraps around hue=0, so keep the two-range OR logic.
//
//Increase the area threshold (< 500) if you see flickering detections on noise.
//
//If you want this as an autonomous-friendly detector (e.g., return LEFT/CENTER/RIGHT based on centroid X), say the word and I’ll add a compact decision helper.
//
//
//
//
//
//
//
//You said:
//want to detect if just if purple or green the code should be able to be simpler if the artifact will be directly infront of the camera
//
//
//Got it—since the game piece will be centered, we can keep it super lean: look only at a small center ROI, threshold in HSV for purple or green, and decide by which mask fills more of that ROI.
//
//Drop this single file into TeamCode/src/main/java/.../opmodes/DetectPurpleGreenOpMode.java. It uses EasyOpenCV and a Logitech USB webcam named “Webcam 1” in your RC config.
//
//java

@TeleOp(name="Color Detector")
public class ColorDetector extends LinearOpMode {

    private OpenCvWebcam webcam;
    private PGPipeline pipeline;

    enum PG { PURPLE, GREEN, NONE }

    @Override
    public void runOpMode() throws InterruptedException {
        int viewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, viewId);

        pipeline = new PGPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                // Use 640x480 for speed; rotate if your camera is sideways
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
            }
        });

        telemetry.addLine("Press PLAY to start detection");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Detected", pipeline.detected);
            telemetry.addData("purple %", "%.1f", pipeline.purplePct * 100.0);
            telemetry.addData("green  %", "%.1f", pipeline.greenPct  * 100.0);
            telemetry.addData("FPS", "%.1f", webcam.getFps());

            // Example: quick action hook
            // if (pipeline.detected == PG.PURPLE) { /* do thing */ }
            // else if (pipeline.detected == PG.GREEN) { /* do other thing */ }

            telemetry.update();
            sleep(20);
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    // --- Minimal pipeline focusing on a center ROI ---
    static class PGPipeline extends OpenCvPipeline {
        // Tunable HSV ranges (OpenCV Hue in [0,180], S/V in [0,255])
        // Purple (magenta-ish cones/pixels). Adjust for your lighting.
        public Scalar purpleLo = new Scalar(130,  80, 80);
        public Scalar purpleHi = new Scalar(160, 255,255);

        // Green game piece
        public Scalar greenLo  = new Scalar(50,   80, 80);
        public Scalar greenHi  = new Scalar(85,  255,255);

        // Decision settings
        public double minFillFraction = 0.05;   // 5% of ROI to count as "present"
        public double purplePct = 0.0, greenPct = 0.0;
        public PG detected = PG.NONE;

        // Mats reused
        private final Mat hsv = new Mat();
        private final Mat maskP = new Mat();
        private final Mat maskG = new Mat();
        private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));

        // ROI set after first frame
        private Rect roi = null;

        @Override
        public Mat processFrame(Mat input) {
            if (input.empty()) return input;

            // Lazy-init ROI: middle 40% width x 40% height
            if (roi == null) {
                int w = input.width();
                int h = input.height();
                int rw = (int)(w * 0.40);
                int rh = (int)(h * 0.40);
                int rx = (w - rw)/2;
                int ry = (h - rh)/2;
                roi = new Rect(rx, ry, rw, rh);
            }

            // Convert and threshold
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, purpleLo, purpleHi, maskP);
            Core.inRange(hsv, greenLo,  greenHi,  maskG);

            // Clean small noise
            Imgproc.morphologyEx(maskP, maskP, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskG, maskG, Imgproc.MORPH_OPEN, kernel);

            // Evaluate only center ROI
            Mat pROI = maskP.submat(roi);
            Mat gROI = maskG.submat(roi);

            double pCount = Core.countNonZero(pROI);
            double gCount = Core.countNonZero(gROI);
            double total  = roi.area();

            purplePct = pCount / total;
            greenPct  = gCount / total;

            // Decide
            boolean purpleSeen = purplePct >= minFillFraction;
            boolean greenSeen  = greenPct  >= minFillFraction;

            if (purpleSeen && greenSeen) {
                detected = (purplePct >= greenPct) ? PG.PURPLE : PG.GREEN;
            } else if (purpleSeen) {
                detected = PG.PURPLE;
            } else if (greenSeen) {
                detected = PG.GREEN;
            } else {
                detected = PG.NONE;
            }

            // Draw ROI + label overlay
            Imgproc.rectangle(input,
                    new Point(roi.x, roi.y),
                    new Point(roi.x + roi.width, roi.y + roi.height),
                    new Scalar(255,255,255), 2);

            String txt = "PG: " + detected + String.format("  P:%.0f%% G:%.0f%%",
                    purplePct*100.0, greenPct*100.0);
            Imgproc.putText(input, txt, new Point(20, 40),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255,255,255), 2);

            return input;
        }

        @Override
        public void onViewportTapped() {
            // Optional: tap to toggle something if you want
        }
    }
}
