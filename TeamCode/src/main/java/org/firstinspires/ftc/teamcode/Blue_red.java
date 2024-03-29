/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.Arm;
import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Blue_red")
public class Blue_red extends LinearOpMode {

    SampleMecanumDrive drive;
    MiniCookies minicookies;
    GetCookies lift;
    Arm arm;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime start = new ElapsedTime();

    final Thread update_it = new Thread() {
        public void run() {
            arm.update();
        }
    };


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        minicookies = new MiniCookies(hardwareMap);
        lift = new GetCookies(hardwareMap);
        arm = new Arm(hardwareMap);


        TrajectorySequence tr1 = drive.trajectorySequenceBuilder(new Pose2d(31.45, -63.28, Math.toRadians(90.00)))
                .addTemporalMarker(0,()->{
                minicookies.startoff();

        })
                .splineTo(new Vector2d(37.89, -18.38), Math.toRadians(94.04))
                .splineToLinearHeading(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)), Math.toRadians(23.79))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)))
                .lineToLinearHeading(new Pose2d(60.23, -12.95, Math.toRadians(0.00)))
                .build();


        TrajectorySequence mid = drive.trajectorySequenceBuilder(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)))
                .lineToLinearHeading(new Pose2d(36.61, -11.83, Math.toRadians(0.00)))
                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)))
                .lineToLinearHeading(new Pose2d(11.61, -13.74, Math.toRadians(0.00)))
                .build();



        drive.setPoseEstimate(tr1.start());

        waitForStart();

        drive.followTrajectorySequence(tr1);

        minicookies.startoff();

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 1.5) {
                lift.up(2);
                minicookies.pick.setPosition(0.660);
            }
            if (runtime.seconds() > 1 && runtime.seconds() < 1.35) {
                minicookies.put();
                minicookies.stack5();
            }

            if (runtime.seconds() > 1.4 && runtime.seconds()<1.6) {
                minicookies.take();
                minicookies.pick.setPosition(0.56);
            }
            if(runtime.seconds()>1.55){
                minicookies.close();
            }
            if (runtime.seconds() > 1.65) {
                lift.down();
            }

            lift.update();
        }


        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.2) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.9) {
                minicookies.load();
            }

            if (runtime.seconds() > 1 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.660);
            }

            if (runtime.seconds() > 2.35 && runtime.seconds() < 2.7) {
                minicookies.put();
                minicookies.stack4();
            }

            if (runtime.seconds() > 2.8 && runtime.seconds()<3.1) {
                minicookies.take();
                minicookies.pick.setPosition(0.56);
            }
            if(runtime.seconds()>2.9){
                minicookies.close();
            }
            if (runtime.seconds() > 3.2) {
                lift.down();

            }

            lift.update();

        }


        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.2) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.9) {
                minicookies.load();
            }

            if (runtime.seconds() > 1 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.660);
            }

            if (runtime.seconds() > 2.35 && runtime.seconds() < 2.7) {
                minicookies.put();
                minicookies.stack3();
            }

            if (runtime.seconds() > 2.8 && runtime.seconds()<3.1) {
                minicookies.take();
                minicookies.pick.setPosition(0.56);


            }
            if(runtime.seconds()>2.9){
                minicookies.close();

            }
            if (runtime.seconds() > 3.2) {
                lift.down();
            }

            lift.update();

        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.2) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.9) {
                minicookies.load();
            }

            if (runtime.seconds() > 1 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.660);
            }

            if (runtime.seconds() > 2.35 && runtime.seconds() < 2.7) {
                minicookies.put();
                minicookies.stack2();
            }

            if (runtime.seconds() > 2.8 && runtime.seconds()<3.1) {
                minicookies.take();
                minicookies.pick.setPosition(0.56);

            }
            if(runtime.seconds()>2.9){
                minicookies.close();

            }
            if (runtime.seconds() > 3.2) {
                lift.down();

            }

            lift.update();

        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.2) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.9) {
                minicookies.load();
            }

            if (runtime.seconds() > 1 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.660);
            }

            if (runtime.seconds() > 2.35 && runtime.seconds() < 2.7) {
                minicookies.put();
                minicookies.stack1();
            }

            if (runtime.seconds() > 2.8 && runtime.seconds()<3.1) {
                minicookies.take();
                minicookies.pick.setPosition(0.56);
            }

            if(runtime.seconds()>2.9){
                minicookies.close();

            }

            if (runtime.seconds() > 3.2) {
                lift.down();
            }

            lift.update();

        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.2) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.9) {
                minicookies.load();
            }

            if (runtime.seconds() > 1 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.660);
            }

            if (runtime.seconds() > 2.35 && runtime.seconds() < 2.7) {
                minicookies.put();
            }

            if (runtime.seconds() > 2.75 && runtime.seconds()<2.95) {
                minicookies.take();
                minicookies.pick.setPosition(0.56);
            }
            if (runtime.seconds() > 3.2) {
                lift.down();
            }

            lift.update();

        }


   /*     runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.2) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.5) {
                minicookies.stack3();
            }

            if (runtime.seconds() > 0.75 && runtime.seconds() < 1.55) {
                minicookies.load();
            }

            if (runtime.seconds() > 1.65 && runtime.seconds() < 3.35) {
                lift.up(2);
                minicookies.pick.setPosition(0.53);
            }
            if (runtime.seconds() > 2.85 && runtime.seconds() < 3.35) {
                minicookies.put();
            }

            if (runtime.seconds() > 3.45) {
                minicookies.take();
                minicookies.pick.setPosition(0.33);
            }
            if (runtime.seconds() > 3.6) {
                lift.down();
            }

            lift.update();

        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.2) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.5) {
                minicookies.stack2();
            }

            if (runtime.seconds() > 0.75 && runtime.seconds() < 1.55) {
                minicookies.load();
            }

            if (runtime.seconds() > 1.65 && runtime.seconds() < 3.35) {
                lift.up(2);
                minicookies.pick.setPosition(0.53);
            }
            if (runtime.seconds() > 2.85 && runtime.seconds() < 3.35) {
                minicookies.put();
            }

            if (runtime.seconds() > 3.45) {
                minicookies.take();
                minicookies.pick.setPosition(0.33);
            }
            if (runtime.seconds() > 3.6) {
                lift.down();
            }

            lift.update();

        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.2) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.5) {
                minicookies.stack1();
            }

            if (runtime.seconds() > 0.75 && runtime.seconds() < 1.55) {
                minicookies.load();
            }

            if (runtime.seconds() > 1.65 && runtime.seconds() < 3.35) {
                lift.up(2);
                minicookies.pick.setPosition(0.53);
            }
            if (runtime.seconds() > 2.85 && runtime.seconds() < 3.35) {
                minicookies.put();
            }

            if (runtime.seconds() > 3.45) {
                minicookies.take();
                minicookies.pick.setPosition(0.33);
            }
            if (runtime.seconds() > 3.6) {
                lift.down();
            }

            lift.update();

        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.5) {
                minicookies.stack5();
            }

            if (runtime.seconds() > 0.6 && runtime.seconds() < 1.4) {
                minicookies.load();
            }

            if (runtime.seconds() > 1.5 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.53);
            }
            if (runtime.seconds() > 2.7 && runtime.seconds() < 3.2) {
                minicookies.put();
            }

            if (runtime.seconds() > 3.3) {
                minicookies.take();
                minicookies.pick.setPosition(0.33);
            }
            if (runtime.seconds() > 3.45) {
                lift.down();
            }

            lift.update();

        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.5) {
                minicookies.stack4();
            }

            if (runtime.seconds() > 0.6 && runtime.seconds() < 1.4) {
                minicookies.load();
            }

            if (runtime.seconds() > 1.5 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.53);
            }
            if (runtime.seconds() > 2.7 && runtime.seconds() < 3.2) {
                minicookies.put();
            }

            if (runtime.seconds() > 3.3) {
                minicookies.take();
                minicookies.pick.setPosition(0.33);
            }
            if (runtime.seconds() > 3.45) {
                lift.down();
            }

            lift.update();


        }


        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.5) {
                minicookies.stack3();
            }

            if (runtime.seconds() > 0.6 && runtime.seconds() < 1.4) {
                minicookies.load();
            }

            if (runtime.seconds() > 1.5 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.53);
            }
            if (runtime.seconds() > 2.7 && runtime.seconds() < 3.2) {
                minicookies.put();
            }

            if (runtime.seconds() > 3.3) {
                minicookies.take();
                minicookies.pick.setPosition(0.33);
            }
            if (runtime.seconds() > 3.45) {
                lift.down();
            }

            lift.update();


        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.5) {
                minicookies.stack2();
            }

            if (runtime.seconds() > 0.6 && runtime.seconds() < 1.4) {
                minicookies.load();
            }

            if (runtime.seconds() > 1.5 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.53);
            }
            if (runtime.seconds() > 2.7 && runtime.seconds() < 3.2) {
                minicookies.put();
            }

            if (runtime.seconds() > 3.3) {
                minicookies.take();
                minicookies.pick.setPosition(0.33);
            }
            if (runtime.seconds() > 3.45) {
                lift.down();
            }

            lift.update();


        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4) {
            if (runtime.seconds() > 0.1 && runtime.seconds() < 0.5) {
                minicookies.stack1();
            }

            if (runtime.seconds() > 0.6 && runtime.seconds() < 1.4) {
                minicookies.load();
            }

            if (runtime.seconds() > 1.5 && runtime.seconds() < 3.2) {
                lift.up(2);
                minicookies.pick.setPosition(0.53);
            }
            if (runtime.seconds() > 2.7 && runtime.seconds() < 3.2) {
                minicookies.put();
            }

            if (runtime.seconds() > 3) {
                minicookies.take();
                minicookies.pick.setPosition(0.33);
            }
            if (runtime.seconds() > 3.45) {
                lift.down();
            }

            lift.update();


        }

       */
    }
}

