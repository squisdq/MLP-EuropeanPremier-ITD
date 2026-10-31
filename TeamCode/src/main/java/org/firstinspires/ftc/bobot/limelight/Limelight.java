package org.firstinspires.ftc.bobot.limelight;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class Limelight {
    public static double CAMERA_HEIGHT_IN = 15.5;
    public static double CAMERA_ANGLE_DEG = 30;

    private final Limelight3A limelight;

    private SampleDetection latestResult = null;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void resetLatestResult() {
        latestResult = null;
    }

    public SampleDetection getSampleDetection() {
         LLResult result = limelight.getLatestResult();
         List<DetectorResult> detections = result.getDetectorResults();

         for (DetectorResult detection : detections) {
             double txRad = Math.toRadians(detection.getTargetXDegrees());
             double tyRad = Math.toRadians(detection.getTargetYDegrees());
             double cameraAngleRad = Math.toRadians(CAMERA_ANGLE_DEG);

             double totalPitchRad = cameraAngleRad + tyRad;

             double dy = CAMERA_HEIGHT_IN * Math.tan(totalPitchRad);
             double dx = dy * Math.tan(txRad);

             latestResult = new SampleDetection(dx, dy, false);
             return new SampleDetection(dx, dy, true);
         }

         return latestResult;
    }

    public enum Sample {
        BLUE(0), RED(1), YELLOW(2);

        final int classID;
        Sample(int classID) {
            this.classID = classID;
        }
    }
}
