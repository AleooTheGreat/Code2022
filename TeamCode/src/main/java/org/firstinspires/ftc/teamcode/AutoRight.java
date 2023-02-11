package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import kotlin.Unit;

@Autonomous(name="AutoDreapta", group="LinearOpMode")
@Disabled

public class AutoRight extends LinearOpMode {
    SampleMecanumDrive drive;
    GetCookies lift;
    MiniCookies sv;

    ElapsedTime seqtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new GetCookies(hardwareMap);
        sv = new MiniCookies(hardwareMap);

        //Up
        TrajectorySequence tr1 = drive.trajectorySequenceBuilder(new Pose2d(34.37, 63.09, Math.toRadians(0.00)))
                .addTemporalMarker(0, () -> {
                    sv.lilpimp.setPosition(0);
                })
                .addTemporalMarker(0.2, ()->{
                    sv.update_servo(0.45);
                })
                .lineTo(new Vector2d(38.28, 24.40))
                .build();


        TrajectorySequence tr2 = drive.trajectorySequenceBuilder(tr1.end())
                .addTemporalMarker(0,()->{
            seq1();
        })
                .waitSeconds(1)
                .lineTo(new Vector2d(32.98, 12.59))
                .addTemporalMarker(1.2,() -> {
                    sv.update_servo(0.10);
                    sv.open();
                })
                .lineTo(new Vector2d(58.00, 14))
                .build();

        TrajectorySequence tr3 = drive.trajectorySequenceBuilder(tr2.end())
                .addTemporalMarker(0, ()->{
                    sv.lilpimp.setPosition(0.0);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(0.15, ()->{
                    sv.update_servo(0.45);
                })
                .splineToLinearHeading(new Pose2d(37.83, 16.99, Math.toRadians(-25.00)), Math.toRadians(20.00))
                .build();



       /* TrajectorySequence tr4 = drive.trajectorySequenceBuilder(tr3.end())
                .addTemporalMarker(0,()->{
                    seq1();
                })
                .waitSeconds(1)
                .addTemporalMarker(1.1, () ->{
                    sv.update_servo(0.075);
                    sv.open();
                })
                .splineToLinearHeading(new Pose2d(58.00, 14, Math.toRadians(0.00)), Math.toRadians(20.00))
                .build();

        TrajectorySequence tr34 = drive.trajectorySequenceBuilder(tr4.end())
                .addTemporalMarker(0, ()->{
                    sv.lilpimp.setPosition(0.0);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(0.15, ()->{
                    sv.update_servo(0.45);
                })
                .splineToLinearHeading(new Pose2d(37.83, 16.99, Math.toRadians(-25.00)), Math.toRadians(20.00))
                .build();

        TrajectorySequence tr5 = drive.trajectorySequenceBuilder(tr34.end())
                .addTemporalMarker(0,()->{
                    seq1();
                })
                .waitSeconds(1)
                .addTemporalMarker(1.1, () ->{
                    sv.update_servo(0.05);
                    sv.open();
                })
                .splineToLinearHeading(new Pose2d(58.00, 14, Math.toRadians(0.00)), Math.toRadians(20.00))
                .build();
/*
        TrajectorySequence tr6 = drive.trajectorySequenceBuilder(new Pose2d(37.83, 16.99, Math.toRadians(-25.00)))
                .addTemporalMarker(0, () ->{
                    sv.update_servo(0.055);
                })
                .splineToLinearHeading(new Pose2d(58.00, 13.03, Math.toRadians(0.00)), Math.toRadians(20.00))
                .build();

        TrajectorySequence tr7 = drive.trajectorySequenceBuilder(new Pose2d(37.83, 16.99, Math.toRadians(-25.00)))
                .addTemporalMarker(0, () ->{
                    sv.update_servo(0.025);
                })

                .splineToLinearHeading(new Pose2d(58.00, 13.03, Math.toRadians(0.00)), Math.toRadians(20.00))
                .addDisplacementMarker(()->{
                    seq1();
                })
                .build();

        TrajectorySequence tr8 = drive.trajectorySequenceBuilder(new Pose2d(37.83, 16.99, Math.toRadians(-25.00)))
                .addTemporalMarker(0, () ->{
                    sv.update_servo(0);
                })
                .splineToLinearHeading(new Pose2d(58.00, 13.03, Math.toRadians(0.00)), Math.toRadians(20.00))
                .build();

         */


        drive.setPoseEstimate(tr1.start());

        waitForStart();

        if(isStopRequested()){
            return;
        }

        drive.followTrajectorySequence(tr1);
        drive.followTrajectorySequence(tr2);
        drive.followTrajectorySequence(tr3);
        /*drive.followTrajectorySequence(tr4);
        drive.followTrajectorySequence(tr34);
        drive.followTrajectorySequence(tr5);
/*
        drive.followTrajectorySequence(tr3);

        seq1();
        sv.open();

        drive.followTrajectorySequence(tr5);

        drive.followTrajectorySequence(tr3);

        seq1();
        sv.open();

        drive.followTrajectorySequence(tr6);

        drive.followTrajectorySequence(tr3);

        seq1();
        sv.open();

        drive.followTrajectorySequence(tr7);

        drive.followTrajectorySequence(tr3);

        seq1();
        sv.open();

        drive.followTrajectorySequence(tr8);
        drive.followTrajectorySequence(tr3);

        seq1();
        sv.open();

        */


    }

    public void seq1() {
        lift.up(1);
        sv.up();
        seqtime.reset();
        while(seqtime.seconds() < 1) {
            if(seqtime.seconds() > 0.5) {
                sv.openup();
            }
            if(seqtime.seconds() > 0.7){
                sv.update_servo(0.45);
            }
            if(seqtime.seconds() > 0.9){
                lift.down();
            }
            lift.update();
        }
    }

}

