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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    // Gamepad 1 (Primary)
    // left stick: drive
    // shoulders: rotate
    // d-pad: arm position
    // y: release block
    // b: reverse drive controls
    // x: spin carousel
    // left bumper:
    // right bumper: eject stuck block from intake

    // Gamepad 2 (Emergency)
    // right stick y: arm target
    // a: Reset arm zero
    // d-pad: Set servo position
    // left stick y: Move servo


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

    I2C
        Control Hub
            0: imu

    Servo
        Control Hub
            0: dumper
    */

    // DriveSimple2
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear OpMode")
public class TeleOp extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx driveFrontLeft = null;
    private DcMotorEx driveBackLeft = null;
    private DcMotorEx driveFrontRight = null;
    private DcMotorEx driveBackRight = null;
    public double TRIGGER_POWER_SCALAR = 0.5;
    private boolean useReversed = false;
    private boolean bPrev = false;

    // Used to spin duck discs
    private DcMotorEx carouselLeft = null;
    private DcMotorEx carouselRight = null;
    private int carouselRightZero = 0;
    private int carouselLeftZero = 0;
    public double carouselSpeed = 1.0;
    private long xPressTime = 0;
    private boolean xPrev = false;

    public double CAROUSEL_STOP_SPEED = 0.5;
    public long CAROUSEL_RAMP_UP_TIME = 500000000;

    private DcMotorEx intake = null;
    public double INTAKE_POWER = 0.5;
    public double INTAKE_EJECT_SPEED = -0.5;


    private int armTarget = 0;
    private DcMotorEx arm = null;

    private Servo dumper = null;

    private boolean leftBumperPrev = false;





    public static boolean within(double val, double target, double tolerance)
    {
        return Math.abs(val - target) <= tolerance;
    }
    public static boolean within(long val, long target, long tolerance)
    {
        return Math.abs(val - target) <= tolerance;
    }

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
        // Set mo
        driveFrontLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveBackRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);

        carouselLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselLeft.setDirection(DcMotor.Direction.FORWARD);
        carouselRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselRight");
        carouselRight.setDirection(DcMotor.Direction.REVERSE);

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        arm = (DcMotorEx)hardwareMap.get(DcMotor.class, "arm");
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dumper = (Servo)hardwareMap.get(Servo.class, "dumper");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down)
            {
                armTarget = 0;
                intake.setPower(-0.1);
            }
            else if (gamepad1.dpad_right)
            {
                armTarget = -600;
                intake.setPower(0.0); // The dumping box can easily lift the intake
            }
            else if (gamepad1.dpad_up)
            {
                armTarget = -900;
                intake.setPower(0.0);
            }

            // Manual arm control
            armTarget += -gamepad2.right_stick_y * 100;

            // Reset zero point on arm
            if (gamepad2.a && arm.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            arm.setTargetPosition(armTarget);

            if (arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            {
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (within(arm.getCurrentPosition(), armTarget, 50))
            {
                if (within(arm.getCurrentPosition(), 0, 20))
                {
                    arm.setPower(0.1); // Power is unneeded if it is going to neutral
                }
                else
                {
                    arm.setPower(0.5);
                }

                if (armTarget == 0)
                {
                    dumper.setPosition(0.0); // Allow intake
                }
            }
            else
            {
                dumper.setPosition(0.15); // Hold block
            }

            if (gamepad1.y)
            {
                dumper.setPosition(0.3); // Release block
            }

            // Manual dumper control
            if (gamepad2.dpad_down)
            {
                dumper.setPosition(0.0); // Allow intake
            }
            else if (gamepad2.dpad_right)
            {
                dumper.setPosition(0.15); // Hold block
            }
            else if (gamepad2.dpad_up)
            {
                dumper.setPosition(0.3); // Release block
            }
            else if (gamepad2.left_stick_y != 0.0)
            {
                dumper.setPosition(dumper.getPosition() - (gamepad2.left_stick_y / 50));
            }





            if (gamepad1.b != bPrev && gamepad1.b)
            {
	            useReversed = !useReversed;
            }
            bPrev = gamepad1.b;
            
            if (useReversed)
            {
                gamepadMove(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.left_trigger * TRIGGER_POWER_SCALAR,
					gamepad1.right_trigger * TRIGGER_POWER_SCALAR);
            }
            else
            {
                gamepadMove(-gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        gamepad1.right_trigger * TRIGGER_POWER_SCALAR,
                        gamepad1.left_trigger * TRIGGER_POWER_SCALAR);
            }

            // On x pressed
            if (gamepad1.x && !xPrev)
            {
                carouselLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                carouselRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                xPressTime = System.nanoTime();
            }

            // On x held
            if (gamepad1.x)
            {
                long currentTime = System.nanoTime();
                double elapsed = currentTime - xPressTime;
                telemetry.addLine("elapsed " + elapsed);

                double power = Math.exp(elapsed/CAROUSEL_RAMP_UP_TIME) - 1;
                carouselLeft.setPower(power);
                carouselRight.setPower(power);
            }

            // On x let go, brake
            if (!gamepad1.x && xPrev)
            {
                carouselLeftZero = carouselLeft.getCurrentPosition();
                carouselLeft.setTargetPosition(carouselLeft.getCurrentPosition());
                carouselLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carouselLeft.setPower(CAROUSEL_STOP_SPEED);

                carouselRightZero = carouselRight.getCurrentPosition();
                carouselRight.setTargetPosition(carouselRight.getCurrentPosition());
                carouselRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carouselRight.setPower(CAROUSEL_STOP_SPEED);
            }
            if (!gamepad1.x)
            {
                carouselLeft.setTargetPosition(carouselLeftZero);
                carouselRight.setTargetPosition(carouselRightZero);
            }

            xPrev = gamepad1.x;


            //carouselLeft.setPower((gamepad1.x ? carouselSpeed : 0.0) - (gamepad1.a ? carouselSpeed : 0.0));
            //carouselRight.setPower((gamepad1.x ? carouselSpeed : 0.0) - (gamepad1.a ? carouselSpeed : 0.0));


            // Test if the left bumper has been pressed down
            boolean leftBumperCurr = gamepad1.left_bumper;
            if (leftBumperCurr && !leftBumperPrev)
            {
                if (intake.getPower() != INTAKE_POWER)
                {
                    intake.setPower(INTAKE_POWER);
                }
                else // Already on, set to off
                {
                    intake.setPower(0.0);
                }
            }
            leftBumperPrev = leftBumperCurr;

            if (intake.getPower() == INTAKE_EJECT_SPEED && !gamepad1.right_bumper)
            {
                intake.setPower(0.0);
            }
            else if (gamepad1.right_bumper)
            {
                intake.setPower(INTAKE_EJECT_SPEED);
            }



            telemetry.addLine("armPos" + arm.getCurrentPosition());
            telemetry.addLine("armTarget " + armTarget);
            telemetry.addLine("xPressTime " + xPressTime);

            telemetry.update();
        }
    }
}
