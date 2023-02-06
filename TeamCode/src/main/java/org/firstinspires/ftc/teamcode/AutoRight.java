package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutoDreapta", group="LinearOpMode")


public class AutoRight extends LinearOpMode {

    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        MiniCookies sv = new MiniCookies();
        sv.init(hardwareMap);

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(35.05, 62.91, Math.toRadians(268.85)))
                .splineTo(new Vector2d(53.43, 12.69), Math.toRadians(-17.10))
                .splineTo(new Vector2d(59.49, 12.51), Math.toRadians(4.40))
                .build();
        drive.setPoseEstimate(untitled0.start());

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(untitled0);
    }
}
