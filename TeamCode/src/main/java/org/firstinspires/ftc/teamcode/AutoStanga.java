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

@Autonomous(name="AutoStanga", group = "LinearOpMode")

public class AutoStanga extends LinearOpMode {

    SampleMecanumDrive drive ;
    GetCookies lift;
    MiniCookies sv;

    ElapsedTime seqtime = new ElapsedTime();

    Queue<TrajectorySequence> trajectoryQueue;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new GetCookies(hardwareMap);
        sv = new MiniCookies(hardwareMap);


        //Up
        TrajectorySequence tr1 = drive.trajectorySequenceBuilder(new Pose2d(34.37, 63.09, Math.toRadians(0.00)))
                .addTemporalMarker(0, () -> {
                    sv.lilpimp.setPosition(0);
                })
                .addTemporalMarker(0.2, () -> {
                    sv.update_servo(0.45);
                })
                .lineTo(new Vector2d(38.2, 24.4))
                .addDisplacementMarker(() -> {
                    seq1();
                })
                .lineTo(new Vector2d(32.98, 12.59))
                .build();

        TrajectorySequence tr2 = drive.trajectorySequenceBuilder(tr1.end())
                .addTemporalMarker(0,() -> {
                    sv.down();
                })
                .waitSeconds(3)
                .addTemporalMarker(2, () -> {
                    sv.open();
                    sv.update_servo(0.12);
                })
                .lineTo(new Vector2d(59.65, 12.8))
                .build();

        TrajectorySequence tr3 = drive.trajectorySequenceBuilder(tr2.end())
                .waitSeconds(0.3)
                .addTemporalMarker(0,()->{
                    sv.close();
                })
                .addTemporalMarker(0.2,()->{
                    sv.update_servo(0.45);
                })
                .splineToLinearHeading(new Pose2d(37.83, 16.99, Math.toRadians(-25.00)), Math.toRadians(20.00))
                .addDisplacementMarker(()->{
                seq1();
                })
                .splineToLinearHeading(new Pose2d(54.00, 13.75, Math.toRadians(0.00)), Math.toRadians(20.00))
                .build();

      /*  TrajectorySequence tr4 = drive.trajectorySequenceBuilder((tr3.end()))
                .waitSeconds(3)
                .addTemporalMarker(1.2,()->{
                    sv.down();
                })
                .addTemporalMarker(2.6,()->{
                            sv.open();
                            sv.update_servo(0.095);
                        })
                .lineTo(new Vector2d(58.00, 13.75))
                .build();

       */



        /*   TrajectorySequence tr5 = drive.trajectorySequenceBuilder(tr3.end())
                .waitSeconds(0.2)
                .addTemporalMarker(0, () -> {
                    sv.lilpimp.setPosition(0.0);
                })
                .waitSeconds(0.15)
                .addTemporalMarker(0.15, () -> {
                    sv.update_servo(0.45);
                })
                .splineToLinearHeading(new Pose2d(37.83, 16.99, Math.toRadians(-25.00)), Math.toRadians(20.00))
                .addDisplacementMarker( () -> {
                    seq1();
                })
                .addDisplacementMarker(()-> {
                    sv.down();
                    sv.update_servo(0.04);
                    sv.open();
                })
                .splineToLinearHeading(new Pose2d(58.00, 13.75, Math.toRadians(0.00)), Math.toRadians(20.00))
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
        //drive.followTrajectorySequence(tr4);
        //drive.followTrajectorySequence(tr3);
    }

    public void seq1() {
        lift.up(1);
        sv.update_servo(0.64);
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
