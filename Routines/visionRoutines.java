package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

public class visionRoutines extends OpMode
{

DcMotor frontLeft, frontRight, backLeft, backRight;
RotationPIDController rotationPID = new RotationPIDController(0.02, 0.0, 0.001);
AprilTagDetection id22 = null;

         public void init() 
    {
        aprilTagWebcam.init(hardwareMap, telemetry);

        intake    = hardwareMap.get(DcMotor.class, "intake");
        indexer   = hardwareMap.get(DcMotor.class, "indexer");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight  = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotationPID.setTolerance(2.0);

        telemetry.addData("Status", "Initialized - waiting for camera...");
        telemetry.update();
    }



    public void Aim() {
        aprilTagWebcam.update();

        List<AprilTagDetection> allTags = aprilTagWebcam.getDetectedTags();
        telemetry.addData("Total Tags Seen", allTags.size());
        for (AprilTagDetection tag : allTags) {
            telemetry.addData("Tag ID", tag.id + (tag.metadata != null ? " (in library)" : " (NOT in library)"));
        }

    
    public void GoToPosition() {
            aprilTagWebcam.update();

            List<AprilTagDetection> allTags = aprilTagWebcam.getDetectedTags();
            telemetry.addData("Total Tags Seen", allTags.size());
            for (AprilTagDetection tag : allTags)
            {
                telemetry.addData("Tag ID", tag.id + (tag.metadata != null ? " (in library)" : " (NOT in library — no pose)"));
            }

            AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificID(22);

            if (id24 != null)
            {
                aprilTagWebcam.displayDetectionTelemetry(id24);

                double targetX = id24.ftcPose.x;
                double targetY = id24.ftcPose.y;

                double moveXPower = movementPID.calculate(targetX);
            // telemetry.addData(/*position difference*/);
                moveX(moveXPower);
                double moveYPower = movementPID.calculate(targetY) ;

                moveY(-moveYPower);
                // FIX 8: Changed bare ftcPose.y / ftcPose.x references to
                //        id24.ftcPose.y / id24.ftcPose.x so they actually
                //        compile and reference the detected tag's pose.
            
                telemetry.addData("Y Position target", String.format("%.1f in", id24.ftcPose.y));
                telemetry.addData("X Position target", String.format("%.1f in", id24.ftcPose.x));

            } else {
                movementPID.reset();
                moveX(0);
                moveY(0);
                telemetry.addData("Tag 24", "Not Seen — motors stopped");
            }

            aprilTagWebcam.displayDetectionTelemetry(id24);
            telemetry.update();
        }
    }

    private void moveX(double speed) {
        backLeftMotor.setPower(-speed);   
        backRightMotor.setPower(speed);  
        frontLeftMotor.setPower(-speed); 
        frontRightMotor.setPower(speed);  
        }


     private void moveY(double speed) {
        backLeftMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        }


        //// dont keep this outside please make it inside!!!
        id22 = aprilTagWebcam.getTagBySpecificID(22);

        if (id22 != null) {
            aprilTagWebcam.displayDetectionTelemetry(id22);

            double currentBearing = id22.ftcPose.bearing;
            double targetBearing  = 0.0;
            double rotationPower  = rotationPID.calculate(currentBearing, targetBearing);
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            telemetry.addData("Bearing to Tag", String.format("%.1f deg", currentBearing));
            telemetry.addData("Rotation Power", String.format("%.3f", rotationPower));
            telemetry.addData("At Target",      rotationPID.isAtTarget(currentBearing, targetBearing));
        } else {
            rotationPID.reset();
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            telemetry.addData("Tag 22", "Not Seen — motors stopped");
        }

        telemetry.update();
    }

}