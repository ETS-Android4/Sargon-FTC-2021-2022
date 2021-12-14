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

@TeleOp(name="TeleOp2", group="Linear OpMode")
public class TeleOp2 extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx driveFrontLeft = null;
    private DcMotorEx driveFrontRight = null;
    private DcMotorEx driveBackLeft = null;
    private DcMotorEx driveBackRight = null;

    private DcMotorEx lift = null;
    private ServoControllerEx dumper = null;

    @Override
    public void runOpMode() {
        driveFrontLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, motorNames[0]);
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight = (DcMotorEx)hardwareMap.get(DcMotor.class, motorNames[1]);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, motorNames[2]);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackRight = (DcMotorEx)hardwareMap.get(DcMotor.class, motorNames[3]);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        lift = (DcMotorEx)hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        dumper = (ServoControllerEx)hardwareMap.get(ServoControllerEx.class, "dumper");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        robot.start();

        while (opModeIsActive()) {
            robot.getDrive().gamepadMove(gamepad1.left_stick_x, gamepad1.left_stick_y,
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
