/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
//import org.firstinspires.ftc.robotcore.external.navigation.

@TeleOp(name = "TeleOpControl", group = "Linear Opmode")
public class TeleOpControl extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx driveFrontLeft = null;
    private DcMotorEx driveFrontRight = null;
    private DcMotorEx driveBackLeft = null;
    private DcMotorEx driveBackRight = null;

    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private DcMotorEx arm = null;

    private Servo claw = null;

    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevY = false;

    private boolean toggleA = false;
    private boolean toggleX = false;
    private boolean toggleY = false;

    private void updateToggle() {


        if (gamepad1.a) {
            prevX = true;
        } else if (!gamepad1.a && prevX) {
            toggleX = !toggleX;
            prevX = false;
        }

        intake.setVelocity(toggleX ? 37000 : 0, AngleUnit.RADIANS);
    }

    private void mecanumHorizontal(double power) {
        driveFrontLeft.setPower(power);
        driveFrontRight.setPower(power);
        driveBackLeft.setPower(-power);
        driveBackRight.setPower(-power);
    }

    private void mecanumVertical(double power) {
        driveFrontLeft.setPower(power);
        driveFrontRight.setPower(-power);
        driveBackLeft.setPower(power);
        driveBackRight.setPower(-power);
    }

    private void mecanumRotate(double power) {
        driveFrontLeft.setPower(-power);
        driveFrontRight.setPower(power);
        driveBackLeft.setPower(power);
        driveBackRight.setPower(-power);
    }

    private void runMecanum(double joyX, double joyY, double triggerL, double triggerR) {
        driveFrontLeft.setPower(joyX + -joyY + -triggerR + triggerL);
        driveFrontRight.setPower(joyX + joyY + triggerR + -triggerL);
        driveBackLeft.setPower(-joyX + -joyY + triggerR + -triggerL);
        driveBackRight.setPower(-joyX + joyY + -triggerR + triggerL);
    }

    private void mecanumHeading() {

    }

    private void updatePreviousGamepad() {
        prevA = gamepad1.a;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
    }

    private void updateToggles() {
        if (gamepad1.x) {
            prevX = true;
        } else if (!gamepad1.x && prevX) {
            toggleX = !toggleX;
            prevX = false;
        }
        intake.setVelocity(toggleX ? 37000 : 0, AngleUnit.RADIANS);

        if (gamepad1.y) {
            prevY = true;
        } else if (!gamepad1.y && prevY) {
            toggleY = !toggleY;
            prevY = false;
        }
        launcher.setVelocity(toggleY ? 37000 : 0, AngleUnit.RADIANS);
    }

    @Override
    public void runOpMode() {
        // Initialize motors
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        claw = hardwareMap.get(Servo.class, "claw");

        driveFrontLeft = hardwareMap.get(DcMotorEx.class, "driveFrontLeft");
        driveFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveFrontRight = hardwareMap.get(DcMotorEx.class, "driveFrontRight");
        driveFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveBackLeft = hardwareMap.get(DcMotorEx.class, "driveBackLeft");
        driveBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveBackRight = hardwareMap.get(DcMotorEx.class, "driveBackRight");
        driveBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            runMecanum(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.left_trigger, gamepad1.right_trigger);
            arm.setVelocity(gamepad1.right_stick_y);

            claw.setPosition((gamepad1.a) ? 0.5 : 1);

            updateToggles();
        }

        telemetry.addData(">", "Ended");
        telemetry.update();
    }

}
