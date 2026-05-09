package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.ArrayList;
import java.util.List;

public class vision {
    public AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    private static final double DECODE_TAG_SIZE_INCHES = 2.0;

    private AprilTagLibrary buildDecodeLibrary() {
        return new AprilTagLibrary.Builder()

                .addTag(24, "Red Goal",            DECODE_TAG_SIZE_INCHES, DistanceUnit.INCH)
                .build();
    }

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(buildDecodeLibrary())
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    public void update() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            detectedTags = new ArrayList<>();
            return;
        }
        List<AprilTagDetection> result = aprilTagProcessor.getDetections();
        detectedTags = (result != null) ? result : new ArrayList<>();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedID) {
        if (detectedID == null) return;
        if (detectedID.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s",
                    detectedID.id, detectedID.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (cm)",
                    detectedID.ftcPose.x,
                    detectedID.ftcPose.y,
                    detectedID.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)",
                    detectedID.ftcPose.pitch,
                    detectedID.ftcPose.roll,
                    detectedID.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f",
                    detectedID.ftcPose.range,
                    detectedID.ftcPose.bearing,
                    detectedID.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown — not in library", detectedID.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)",
                    detectedID.center.x, detectedID.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificID(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) return detection;
        }
        return null;
    }
    
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }  
    
   

}