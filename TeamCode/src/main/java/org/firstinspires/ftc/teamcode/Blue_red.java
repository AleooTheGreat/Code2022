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

import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Blue_red")

public class Blue_red extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MiniCookies minicookies = new MiniCookies(hardwareMap);
        GetCookies lift = new GetCookies(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();


        TrajectorySequence tr1 = drive.trajectorySequenceBuilder(new Pose2d(31.45, -63.28, Math.toRadians(90.00)))
                .splineTo(new Vector2d(37.89, -18.38), Math.toRadians(94.04))
                .splineToLinearHeading(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)), Math.toRadians(23.79))
                .build();


        drive.setPoseEstimate(tr1.start());

        waitForStart();


        if (!isStopRequested()) {

             drive.followTrajectorySequence(tr1);

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 2) {
                if (runtime.seconds() > 0.1 && runtime.seconds() < 1) {
                    lift.up(2);
                    minicookies.pick.setPosition(0.38);
                }
                if (runtime.seconds() > 0.85 && runtime.seconds() < 1.15) {
                    minicookies.put();
                }

                if (runtime.seconds() > 1.25) {
                    minicookies.take();
                    minicookies.pick.setPosition(0.2);
                }
                if (runtime.seconds() > 1.55) {
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

                if (runtime.seconds() > 1.5 && runtime.seconds() < 3) {
                    lift.up(2);
                    minicookies.pick.setPosition(0.38);
                }
                if (runtime.seconds() > 2.35 && runtime.seconds() < 3) {
                    minicookies.put();
                }

                if (runtime.seconds() > 3) {
                    minicookies.take();
                    minicookies.pick.setPosition(0.2);
                }
                if (runtime.seconds() > 3.2) {
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

                if (runtime.seconds() > 1.5 && runtime.seconds() < 3) {
                    lift.up(2);
                    minicookies.pick.setPosition(0.38);
                }
                if (runtime.seconds() > 2.35 && runtime.seconds() < 3) {
                    minicookies.put();
                }

                if (runtime.seconds() > 3) {
                    minicookies.take();
                    minicookies.pick.setPosition(0.2);
                }
                if (runtime.seconds() > 3.2) {
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

                if (runtime.seconds() > 1.5 && runtime.seconds() < 3) {
                    lift.up(2);
                    minicookies.pick.setPosition(0.38);
                }
                if (runtime.seconds() > 2.35 && runtime.seconds() < 3) {
                    minicookies.put();
                }

                if (runtime.seconds() > 3) {
                    minicookies.take();
                    minicookies.pick.setPosition(0.2);
                }
                if (runtime.seconds() > 3.2) {
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

                if (runtime.seconds() > 1.5 && runtime.seconds() < 3) {
                    lift.up(2);
                    minicookies.pick.setPosition(0.38);
                }
                if (runtime.seconds() > 2.35 && runtime.seconds() < 3) {
                    minicookies.put();
                }

                if (runtime.seconds() > 3) {
                    minicookies.take();
                    minicookies.pick.setPosition(0.2);
                }
                if (runtime.seconds() > 3.2) {
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

                if (runtime.seconds() > 1.5 && runtime.seconds() < 3) {
                    lift.up(2);
                    minicookies.pick.setPosition(0.38);
                }
                if (runtime.seconds() > 2.35 && runtime.seconds() < 3) {
                    minicookies.put();
                }

                if (runtime.seconds() > 3) {
                    minicookies.take();
                    minicookies.pick.setPosition(0.2);
                }
                if (runtime.seconds() > 3.2) {
                    lift.down();
                }

                lift.update();


            }



        }
    }

}