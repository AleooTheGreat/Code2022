package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Queue;

@Autonomous(name="AutoRight", group = "LinearOpMode")

public class AutoRight extends LinearOpMode {

    SampleMecanumDrive drive ;
    GetCookies lift;
    MiniCookies sv;
    ElapsedTime seqtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new GetCookies(hardwareMap);
        sv = new MiniCookies(hardwareMap);



        //Up
        TrajectorySequence tr1 = drive.trajectorySequenceBuilder(new Pose2d(34.37, -63.09, Math.toRadians(0.00)))
                .addTemporalMarker(0, () -> {
                    sv.lilpimp.setPosition(0);
                })
                .addTemporalMarker(0.2, () -> {
                    sv.update_servo(0.57);
                })
                .lineToLinearHeading(new Pose2d(39, -23.7, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> {
                    seq1();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(33, -13.00, Math.toRadians(0.00)))
                .addDisplacementMarker(()->{
                    sv.open();
                    sv.update_servo(0.22);
                })
                .lineToLinearHeading(new Pose2d(59.10, -12.5, Math.toRadians(0.00)))
                .build();


        TrajectorySequence tr2 = drive.trajectorySequenceBuilder(tr1.end())
                .waitSeconds(0.5)
                .addTemporalMarker(0,() -> {
                    sv.close();
                })
                .addTemporalMarker(0.35, () -> {
                    sv.update_servo(0.57);
                })
                .splineToLinearHeading(new Pose2d(39.00, -16.69, Math.toRadians(25.00)), Math.toRadians(-25.00))
                .addDisplacementMarker(()->{
                    seq1();
                })
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(48.00, -12.20, Math.toRadians(0.00)), Math.toRadians(-12.00))
                .addDisplacementMarker(()->{
                    sv.open();
                    sv.update_servo(0.20);
                })
                .splineToLinearHeading(new Pose2d(59.10, -12.50, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();

        TrajectorySequence tr3 = drive.trajectorySequenceBuilder(tr2.end())
                .waitSeconds(0.5)
                .addTemporalMarker(0,() -> {
                    sv.close();
                })
                .addTemporalMarker(0.3, () -> {
                    sv.update_servo(0.57);
                })
                .splineToLinearHeading(new Pose2d(39.00, -16.69, Math.toRadians(25.00)), Math.toRadians(-25.00))
                .addDisplacementMarker(()->{
                    seq1();
                })
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(48.00, -12.20, Math.toRadians(0.00)), Math.toRadians(-12.00))
                .addDisplacementMarker(()->{
                    sv.open();
                    sv.update_servo(0.18);
                })
                .splineToLinearHeading(new Pose2d(59.10, -12.50, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();


        TrajectorySequence tr4 = drive.trajectorySequenceBuilder((tr3.end()))
                .waitSeconds(0.5)
                .addTemporalMarker(0,() -> {
                    sv.close();
                })
                .addTemporalMarker(0.3, () -> {
                    sv.update_servo(0.57);
                })
                .splineToLinearHeading(new Pose2d(39.00, -16.69, Math.toRadians(25.00)), Math.toRadians(-25.00))
                .addDisplacementMarker(()->{
                    seq1();
                })
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(48.00, -12.20, Math.toRadians(0.00)), Math.toRadians(-12.00))
                .addDisplacementMarker(()->{
                    sv.open();
                    sv.update_servo(0.16);
                })
                .splineToLinearHeading(new Pose2d(59.10, -12.50, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();


        drive.setPoseEstimate(tr1.start());

        waitForStart();

        if(isStopRequested()){
            return;
        }

        drive.followTrajectorySequence(tr1);
        drive.followTrajectorySequence(tr2);
        drive.followTrajectorySequence(tr3);
        drive.followTrajectorySequence(tr4);
    }

    public void seq1() {
        lift.up(1);
        sv.update_servo(0.78);
        seqtime.reset();
        while(seqtime.seconds() < 1) {
            if(seqtime.seconds() > 0.6) {
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
}

