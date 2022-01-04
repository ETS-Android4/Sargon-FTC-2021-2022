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


    /* Configuration

    Control Hub
    0: driveBackLeft
    1: driveFrontLeft
    2: carouselLeft
    3: intake

    Expansion Hub
    0: driveBackRight
    1: driveFrontRight
    2: carouselRight
    3: arm

    */

    // DriveSimple2
@TeleOp(name="TeleOp2", group="Linear OpMode")
public class TeleOp2 extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx driveFrontLeft = null;
    private DcMotorEx driveBackLeft = null;
    private DcMotorEx driveFrontRight = null;
    private DcMotorEx driveBackRight = null;

    // Used to spin duck discs
    private DcMotorEx carouselLeft = null;
    private DcMotorEx carouselRight = null;

    private DcMotorEx intake = null;

    int positionTargetAtZero = 0;
    int cyclesAtZero = 0;
    boolean yPrev = false;
    boolean isBrakingActive = false;
    private DcMotorEx arm = null;

    private Servo dumper = null;

    private boolean leftBumperPrev = false;


    public void gamepadMove(double joyX, double joyY, double triggerL, double triggerR) {

        // Tank Drive
        /*
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
        */

        // Mecanum Drive
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
    }

    @Override
    public void runOpMode() {
        driveFrontLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        carouselLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselLeft.setDirection(DcMotor.Direction.FORWARD);
        carouselRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselRight");
        carouselRight.setDirection(DcMotor.Direction.REVERSE);

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        arm = (DcMotorEx)hardwareMap.get(DcMotor.class, "arm");
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dumper = (Servo)hardwareMap.get(Servo.class, "dumper");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            gamepadMove(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.left_trigger, gamepad1.right_trigger);

            carouselLeft.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            carouselRight.setPower(gamepad1.left_trigger - gamepad1.right_trigger);


            // Test if the left bumper has been pressed down
            boolean leftBumperCurr = gamepad1.left_bumper;
            if (leftBumperCurr && !leftBumperPrev)
            {
                if (intake.getPower() == 0.0)
                {
                    intake.setPower(1.0);
                }
                else // Already on, set to off
                {
                    intake.setPower(0.0);
                }
            }
            leftBumperPrev = leftBumperCurr;

            if (intake.getPower() == -1.0 && !gamepad1.right_bumper)
            {
                intake.setPower(0.0);
            }
            else if (gamepad1.right_bumper)
            {
                intake.setPower(-1.0);
            }

            if (gamepad1.dpad_down)
            {
                dumper.setPosition(1.0);
            }
            else if (gamepad1.dpad_up)
            {
                dumper.setPosition(0.0);
            }



            if (-gamepad1.right_stick_y == 0)
            {
                cyclesAtZero++;

                if (arm.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                    arm.setPower(0.0);
                }
            }
            else
            {
                if (arm.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
                {
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                positionTargetAtZero = arm.getCurrentPosition();
                arm.setPower(-gamepad1.right_stick_y);
                cyclesAtZero = 0;
            }

            boolean yCurr = gamepad1.y;
            if (yCurr && !yPrev)
            {
                if (isBrakingActive)
                {
                    isBrakingActive = false;

                    if (arm.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
                    {
                        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        arm.setPower(-gamepad1.right_stick_y);
                    }
                }
                else
                {
                    isBrakingActive = true;
                }
            }
            yPrev = gamepad1.y;

            if (isBrakingActive)
            {
                if (cyclesAtZero >= 10 && arm.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                    positionTargetAtZero = arm.getCurrentPosition();
                    arm.setTargetPosition(positionTargetAtZero);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.1);
                }
                else if (cyclesAtZero >= 10) {
                    arm.setTargetPosition(positionTargetAtZero);
                }
            }
            telemetry.addLine("IsBrakingActive " + isBrakingActive);
            telemetry.addLine("cyclesAtZero " + cyclesAtZero);

            telemetry.update();
        }
    }
}
