/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCv;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoStanga;
import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Blue extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 0;
    int MIDDLE = 4;
    int RIGHT = 2;

    AutoStanga autost;
    AprilTagDetection tagOfInterest = null;

    SampleMecanumDrive drive ;
    GetCookies lift;
    MiniCookies sv;
    ElapsedTime seqtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new GetCookies(hardwareMap);
        sv = new MiniCookies(hardwareMap);


        //Up
        TrajectorySequence tr1 = drive.trajectorySequenceBuilder(new Pose2d(34.37, 63.09, Math.toRadians(0.00)))
                .addTemporalMarker(0, () -> {
                    sv.lilpimp.setPosition(0);
                })
                .addTemporalMarker(0.2, () -> {
                    sv.update_servo(0.57);
                })
                .lineToLinearHeading(new Pose2d(39, 23.7, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> {
                    seq1();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(33, 13.00, Math.toRadians(0.00)))
                .addDisplacementMarker(()->{
                    sv.open();
                    sv.update_servo(0.22);
                })
                .lineToLinearHeading(new Pose2d(59.10, 12.5, Math.toRadians(0.00)))
                .build();


        TrajectorySequence tr2 = drive.trajectorySequenceBuilder(tr1.end())
                .waitSeconds(0.5)
                .addTemporalMarker(0,() -> {
                    sv.close();
                })
                .addTemporalMarker(0.35, () -> {
                    sv.update_servo(0.57);
                })
                .splineToLinearHeading(new Pose2d(39.00, 16.69, Math.toRadians(-25.00)), Math.toRadians(25.00))
                .addDisplacementMarker(()->{
                    seq1();
                })
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(48.00, 12.20, Math.toRadians(0.00)), Math.toRadians(12.00))
                .addDisplacementMarker(()->{
                    sv.open();
                    sv.update_servo(0.20);
                })
                .splineToLinearHeading(new Pose2d(59.10, 12.50, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();

        TrajectorySequence tr3 = drive.trajectorySequenceBuilder(tr2.end())
                .waitSeconds(0.5)
                .addTemporalMarker(0,() -> {
                    sv.close();
                })
                .addTemporalMarker(0.3, () -> {
                    sv.update_servo(0.57);
                })
                .splineToLinearHeading(new Pose2d(39.00, 16.69, Math.toRadians(-25.00)), Math.toRadians(25.00))
                .addDisplacementMarker(()->{
                    seq1();
                })
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(48.00, 12.20, Math.toRadians(0.00)), Math.toRadians(12.00))
                .addDisplacementMarker(()->{
                    sv.open();
                    sv.update_servo(0.18);
                })
                .splineToLinearHeading(new Pose2d(59.10, 12.50, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();


        TrajectorySequence tr4 = drive.trajectorySequenceBuilder((tr3.end()))
                .waitSeconds(0.5)
                .addTemporalMarker(0,() -> {
                    sv.close();
                })
                .addTemporalMarker(0.3, () -> {
                    sv.update_servo(0.57);
                })
                .splineToLinearHeading(new Pose2d(39.00, 16.69, Math.toRadians(-25.00)), Math.toRadians(25.00))
                .addDisplacementMarker(()->{
                    seq1();
                })
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(48.00, 12.20, Math.toRadians(0.00)), Math.toRadians(12.00))
                .addDisplacementMarker(()->{
                    sv.open();
                    sv.update_servo(0.16);
                })
                .splineToLinearHeading(new Pose2d(59.2, 12.50, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();

        TrajectorySequence tr5 = drive.trajectorySequenceBuilder((tr3.end()))
                .waitSeconds(0.5)
                .addTemporalMarker(0,() -> {
                    sv.close();
                })
                .addTemporalMarker(0.3, () -> {
                    sv.update_servo(0.57);
                })
                .splineToLinearHeading(new Pose2d(39.00, 16.69, Math.toRadians(-25.00)), Math.toRadians(25.00))
                .addDisplacementMarker(()->{
                    seq1();
                })
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(48.00, 12.20, Math.toRadians(0.00)), Math.toRadians(12.00))
                .addDisplacementMarker(()->{
                    sv.open();
                    sv.update_servo(0.14);
                })
                .splineToLinearHeading(new Pose2d(59.25, 12.50, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();

        TrajectorySequence trmid = drive.trajectorySequenceBuilder(tr4.end())

                .lineToLinearHeading(new Pose2d(34.86, 11.75, Math.toRadians(0.00)))
                .build();

        TrajectorySequence trright = drive.trajectorySequenceBuilder(tr4.end())
                .lineToLinearHeading(new Pose2d(11.42, 10.26, Math.toRadians(0.00)))
                .build();



        drive.setPoseEstimate(tr1.start());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            drive.followTrajectorySequence(tr1);
            drive.followTrajectorySequence(tr2);
            drive.followTrajectorySequence(tr3);
            drive.followTrajectorySequence(tr4);
            drive.followTrajectorySequence(tr5);

        }else if(tagOfInterest.id == MIDDLE){
            drive.followTrajectorySequence(tr1);
            drive.followTrajectorySequence(tr2);
            drive.followTrajectorySequence(tr3);
            drive.followTrajectorySequence(tr4);
            drive.followTrajectorySequence(tr5);
            drive.followTrajectorySequence(trmid);



        }else if(tagOfInterest.id == RIGHT){
            drive.followTrajectorySequence(tr1);
            drive.followTrajectorySequence(tr2);
            drive.followTrajectorySequence(tr3);
            drive.followTrajectorySequence(tr4);
            drive.followTrajectorySequence(tr5);
            drive.followTrajectorySequence(trright);

        }

    }
    public void seq1() {
        lift.up(1);
        sv.update_servo(0.78);
        seqtime.reset();
        while(seqtime.seconds() < 1) {
            if(seqtime.seconds() > 0.45) {
                sv.openup();
            }
            if(seqtime.seconds() > 0.8){
                sv.update_servo(0.67);
            }
            if(seqtime.seconds() > 0.95){
                lift.down();
            }
            lift.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}