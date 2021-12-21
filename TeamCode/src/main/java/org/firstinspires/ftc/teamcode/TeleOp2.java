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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

    // Drive left stick
    // Toggle intake with left for speed, right for direction bumpers
    // Carousel triggers
    // Dumper d-pad y
    // Lifter right stick y

@TeleOp(name="TeleOp2", group="Linear OpMode")
public class TeleOp2 extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx driveLeft = null;
    private DcMotorEx driveRight = null;

    private DcMotorEx carouselLeft = null;
    private DcMotorEx carouselRight = null;
    private DcMotorEx intake = null;
    private DcMotorEx lifter = null;
    private Servo dumper = null;

    private boolean leftBumperPrev = false;


    public void gamepadMove(double joyX, double joyY, double triggerL, double triggerR) {
        double r = Math.hypot(joyX, joyY);
        double robotAngle = Math.atan2(joyY, joyX);
        double yaw = triggerR - triggerL;

        double drive = joyY;
        double strafe = -joyX;
        double twist = triggerL - triggerR;

        double frontLeftPower = drive + strafe + twist;
        double frontRightPower = -drive + strafe + twist;
        double backLeftPower = drive - strafe + twist;
        double backRightPower = -drive - strafe + twist;

        driveLeft.setPower(- gamepad1.left_stick_y/2 + gamepad1.left_stick_x/2);
        driveRight.setPower(- gamepad1.left_stick_y/2 - gamepad1.left_stick_x/2);


        /*
        double r = Math.hypot(joyX, joyY);
        double robotAngle = Math.atan2(joyY, joyX);
        double yaw = triggerR - triggerL;
        //double frontLeftPower = (r * Math.cos(robotAngle)) + yaw;
        //double frontRightPower = (r * Math.sin(robotAngle)) - yaw;
        //double backLeftPower = (r * Math.sin(robotAngle)) + yaw;
        //double backRightPower = (r * Math.cos(robotAngle)) - yaw;

        double drive = joyY;
        double strafe = -joyX;
        double twist = triggerL - triggerR;

        double frontLeftPower = drive + strafe + twist;
        double frontRightPower = -drive + strafe + twist;
        double backLeftPower = drive - strafe + twist;
        double backRightPower = -drive - strafe + twist;

        //driveFrontLeft.setPower(frontLeftPower);
        //driveFrontRight.setPower(frontRightPower);
        //driveBackLeft.setPower(backLeftPower);
        //driveBackRight.setPower(backRightPower);*/
    }

    @Override
    public void runOpMode() {
        driveLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveLeft");
        driveLeft.setDirection(DcMotor.Direction.FORWARD);
        driveRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveRight");
        driveRight.setDirection(DcMotor.Direction.REVERSE);

        carouselLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselLeft.setDirection(DcMotor.Direction.FORWARD);
        carouselRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselRight");
        carouselRight.setDirection(DcMotor.Direction.REVERSE);
        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        lifter = (DcMotorEx)hardwareMap.get(DcMotor.class, "lifter");

        dumper = (Servo)hardwareMap.get(Servo.class, "dumper");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            gamepadMove(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.left_trigger, gamepad1.right_trigger);

            // For autonomous mode
            //double heading = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            //double power = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //mecanumDrive(heading, power);

            //mecanumRotate((gamepad1.left_trigger - gamepad1.right_trigger) * Math.PI,
            //        1.0);



            boolean leftBumperCurr = gamepad1.left_bumper;
            if (leftBumperCurr && !leftBumperPrev && intake.getPower() == 0.0)
            {
                if (gamepad1.right_bumper)
                {
                    intake.setPower(-1.0);
                }
                else
                {
                    intake.setPower(1.0);
                }
            }
            else if (leftBumperCurr && !leftBumperPrev && Math.abs(intake.getPower()) != 0.0) {
                intake.setPower(0.0);
            }
            leftBumperPrev = leftBumperCurr;

            if (gamepad1.dpad_down)
            {
                dumper.setPosition(1.0);
            }
            else if (gamepad1.dpad_up)
            {
                dumper.setPosition(0.0);
            }

            carouselLeft.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            carouselRight.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            lifter.setPower(-gamepad1.right_stick_y);

            telemetry.update();
        }
    }
}
