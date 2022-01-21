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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Stats", group="Linear OpMode")
public class Stats extends LinearOpMode {
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

    private DcMotorEx intake = null;


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
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dumper = (Servo)hardwareMap.get(Servo.class, "dumper");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("armPos" + arm.getCurrentPosition());
            telemetry.addLine("armTarget " + armTarget);
            telemetry.addLine("armVelocity" + arm.getVelocity());

            telemetry.addLine("dumperPos " + dumper.getPosition());

            telemetry.update();
        }
    }
}
