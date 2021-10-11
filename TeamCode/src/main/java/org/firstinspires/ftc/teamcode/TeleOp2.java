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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="TeleOp2", group="Linear OpMode")
public class TeleOp2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor driveFrontLeft = null;
    private DcMotor driveFrontRight = null;
    private DcMotor driveBackLeft = null;
    private DcMotor driveBackRight = null;

    private void mecanumDrive(double direction, double power)
    {
        double x = power * Math.cos(direction);
        double y = power * Math.sin(direction);

        double drive = -y;
        double strafe = -x;

        double frontLeftPower = drive + strafe;
        double frontRightPower = -drive + strafe;
        double backLeftPower = drive - strafe;
        double backRightPower = -drive - strafe;

        driveFrontLeft.setPower(frontLeftPower);
        driveFrontRight.setPower(frontRightPower);
        driveBackLeft.setPower(backLeftPower);
        driveBackRight.setPower(backRightPower);
    }

    // Direction: Radians between -PI and PI
    // Power: Double between 0 and 1
    private void mecanumRotate(double direction, double power)
    {
        driveFrontLeft.setPower((direction / Math.PI) * power);
        driveFrontRight.setPower((direction / Math.PI) * power);
        driveBackLeft.setPower((direction / Math.PI) * power);
        driveBackRight.setPower((direction / Math.PI) * power);
    }

    private void runMecanum(double joyX, double joyY, double triggerL, double triggerR) {
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

        driveFrontLeft.setPower(frontLeftPower);
        driveFrontRight.setPower(frontRightPower);
        driveBackLeft.setPower(backLeftPower);
        driveBackRight.setPower(backRightPower);

        telemetry.addData("Angle", "r %f, ra %f, Yaw %f", r, robotAngle, yaw);
        telemetry.addData("Motors", "0 %f, 1 %f, 2 %f, 3 %f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Controller", "X %f, Y %f, L %f, R %f",
                joyX, joyY,
                triggerL, triggerR);
        telemetry.addData("Vals", "drive %f, strafe %f, twist %f",
                drive, strafe, twist);
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);

        while (opModeIsActive()) {
            runMecanum(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.left_trigger, gamepad1.right_trigger);

            // For autonomous mode
            //double heading = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            //double power = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //mecanumDrive(heading, power);

            //mecanumRotate((gamepad1.left_trigger - gamepad1.right_trigger) * Math.PI,
            //        1.0);

            telemetry.update();
        }
    }
}
