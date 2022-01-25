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

import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH;
import static org.firstinspires.ftc.teamcode.Constants.ARM_LOW;
import static org.firstinspires.ftc.teamcode.Constants.ARM_MANUAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.Constants.CAROUSEL_SPEED_CAP;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_HOLD;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_OPEN;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_RELEASE;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER_EJECT;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_POWER_SCALAR;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

import java.util.Timer;
import java.util.TimerTask;

    // Drive left stick
    // Toggle intake with left for speed, right for direction bumpers
    // Carousel triggers
    // Dumper d-pad y
    // Lifter right stick y

    // Gamepad 1 (Primary)
    // left stick: drive
    // right stick y: capping servo (UNUSED)
    // shoulders: rotate
    // d-pad: arm position:
        // Up: high level
        // Down: ready for intake
        // Left: eject block with servo
        // Right: low level
    // a: release block
    // b: reverse drive controls
    // x: spin carousel
    // y: spin carousel reverse
    // left bumper: activate intake
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

    // Config: DriveSimple2

@com.acmerobotics.dashboard.config.Config
public abstract class TeleOpBase extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    //private MotorEx driveFrontLeft = null;
    //private MotorEx driveBackLeft = null;
    //private MotorEx driveFrontRight = null;
    //private MotorEx driveBackRight = null;
    private SampleMecanumDriveCancelable drive;
    private boolean useReversed = true;

    // Used to spin duck discs
    private DcMotorEx carouselLeft = null;
    private DcMotorEx carouselRight = null;

    private DcMotorEx intake = null;

    private int armTarget = 0;
    private DcMotorEx arm = null;


    private Servo dumper = null;


    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);


    Gamepad g1Prev;
    Gamepad g2Prev;

    GamepadEx p1;
    GamepadEx p2;



    public static boolean within(double val, double target, double tolerance)
    {
        return Math.abs(val - target) <= tolerance;
    }
    public static boolean within(long val, long target, long tolerance)
    {
        return Math.abs(val - target) <= tolerance;
    }

    /*public void gamepadMove(double joyX, double joyY, double triggerL, double triggerR) {
        // Mecanum Drive
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

        driveFrontLeft.motor.setPower(frontLeftPower);
        driveFrontRight.motor.setPower(frontRightPower);
        driveBackLeft.motor.setPower(backLeftPower);
        driveBackRight.motor.setPower(backRightPower);
    }*/

    public void initMotors()
    {
        /*driveFrontLeft = new MotorEx(hardwareMap, "driveFrontLeft");
        driveFrontLeft.setInverted(false);
        driveBackLeft = new MotorEx(hardwareMap, "driveBackLeft");
        driveBackLeft.setInverted(false);
        driveFrontRight = new MotorEx(hardwareMap, "driveFrontRight");
        driveFrontRight.setInverted(false);
        driveBackRight = new MotorEx(hardwareMap, "driveBackRight");
        driveBackRight.setInverted(false);*/

        //MecanumDrive mecanum = new MecanumDrive(driveFrontLeft, driveFrontRight, driveBackLeft, driveBackRight);

        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(Constants.blueDuckStartingPose);

        carouselLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselLeft.setDirection(DcMotor.Direction.FORWARD);
        carouselLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselRight");
        carouselRight.setDirection(DcMotor.Direction.REVERSE);
        carouselRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        arm = (DcMotorEx)hardwareMap.get(DcMotor.class, "arm");
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dumper = (Servo)hardwareMap.get(Servo.class, "dumper");
    }

    void runArm()
    {
        if (gamepad1.dpad_down)
        {
            armTarget = 0;
            intake.setPower(-0.1);

            Timer timer = new Timer();
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    if (within(intake.getPower(), -0.1, 0.01)) {
                        intake.setPower(0.0);

                    }
                }
            }, 1*1000);
        }
        else if (gamepad1.dpad_up)
        {
            armTarget = ARM_HIGH;
            intake.setPower(0.0); // The dumping box can easily lift the intake
        }
        else if (gamepad1.dpad_right)
        {
            armTarget = ARM_LOW;
            intake.setPower(0.0);
        }

        // Manual arm control
        armTarget += -gamepad1.right_stick_y * ARM_MANUAL_MULTIPLIER;
        armTarget += -gamepad2.right_stick_y * ARM_MANUAL_MULTIPLIER;

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

        if (!within(arm.getCurrentPosition(), armTarget, 100) && armTarget != 0)
        {
            dumper.setPosition(DUMPER_HOLD); // Hold block
        }

        if (within(arm.getCurrentPosition(), 0, 20) && armTarget == 0)
        {
            arm.setPower(0.0); // Power is unneeded if it is going to neutral and near it
        }
        else if (arm.getTargetPosition() == 0)
        {
            //arm.setPower(0.1);
            arm.setVelocity(400);
        }
        else
        {
            //arm.setPower(0.5);
            arm.setVelocity(800);
        }
    }

    //todo: simplify
    void runDumper()
    {
        if (armTarget != 0)
        {
            if (armTarget != 0) {
                if (p1.wasJustPressed(Button.A)) {
                    dumper.setPosition(DUMPER_RELEASE);
                } else if (!gamepad1.a && g1Prev.a) {
                    dumper.setPosition(DUMPER_OPEN);
                }
            }

        }
        else if (gamepad1.a)
        {
            dumper.setPosition(DUMPER_HOLD);
        }
        else
        {
            //dumper.setPosition(DUMPER_HOLD);
        }

        // Manual dumper control
        if (gamepad1.dpad_left)
        {
            dumper.setPosition(DUMPER_OPEN);
        }
        else if (gamepad2.dpad_down)
        {
            dumper.setPosition(DUMPER_OPEN); // Allow intake
        }
        else if (gamepad2.dpad_right)
        {
            dumper.setPosition(DUMPER_HOLD); // Hold block
        }
        else if (gamepad2.dpad_up)
        {
            dumper.setPosition(DUMPER_RELEASE); // Release block
        }
        else if (gamepad2.left_stick_y != 0.0)
        {
            dumper.setPosition(dumper.getPosition() - (gamepad2.left_stick_y / 50));
        }
    }

    void runDrive()
    {
        if (p1.wasJustPressed(Button.B))
        {
            useReversed = !useReversed;
        }

        if (useReversed)
        {
            //gamepadMove(gamepad1.left_stick_x, gamepad1.left_stick_y,
            //        gamepad1.right_trigger * TRIGGER_POWER_SCALAR,
            //        gamepad1.left_trigger * TRIGGER_POWER_SCALAR);
        }
        else
        {
            //gamepadMove(-gamepad1.left_stick_x, -gamepad1.left_stick_y,
            //        gamepad1.right_trigger * TRIGGER_POWER_SCALAR,
            //        gamepad1.left_trigger * TRIGGER_POWER_SCALAR);
        }
    }

    void runDriveSmart()
    {
        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();

        switch (currentMode) {
            case DRIVER_CONTROL:
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                gamepad1.left_trigger - gamepad1.right_trigger
                        )
                );

                if (gamepad1.b) {
                    // If the A button is pressed on gamepad1, we generate a splineTo()
                    // trajectory on the fly and follow it
                    // We switch the state to AUTOMATIC_CONTROL

                    Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                            .splineTo(targetAVector, targetAHeading)
                            .build();

                    drive.followTrajectoryAsync(traj1);

                    currentMode = Mode.AUTOMATIC_CONTROL;
                } else if (gamepad1.x) {
                    // If the B button is pressed on gamepad1, we generate a lineTo()
                    // trajectory on the fly and follow it
                    // We switch the state to AUTOMATIC_CONTROL

                    Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                            .lineTo(targetBVector)
                            .build();

                    drive.followTrajectoryAsync(traj1);

                    currentMode = Mode.AUTOMATIC_CONTROL;
                } else if (gamepad1.y) {
                    // If Y is pressed, we turn the bot to the specified angle to reach
                    // targetAngle (by default, 45 degrees)

                    drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                    currentMode = Mode.AUTOMATIC_CONTROL;
                }
                break;
            case AUTOMATIC_CONTROL:
                // If x is pressed, we break out of the automatic following
                if (gamepad1.x) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }

                // If drive finishes its task, cede control to the driver
                if (!drive.isBusy()) {
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;
        }
    }

    void runCarousel()
    {
        carouselLeft.setPower(gamepad1.x ? CAROUSEL_SPEED_CAP : 0.0 -
                (gamepad1.y ? CAROUSEL_SPEED_CAP : 0.0));
        carouselRight.setPower(gamepad1.x ? CAROUSEL_SPEED_CAP : 0.0 -
                (gamepad1.y ? CAROUSEL_SPEED_CAP : 0.0));
    }

    void runIntake()
    {
        boolean leftBumperCurr = gamepad1.left_bumper;
        if (leftBumperCurr && !g1Prev.left_bumper)
        {
            if (!within(intake.getPower(), INTAKE_POWER, 0.01))
            {
                intake.setPower(INTAKE_POWER);
            }
            else // Already on, set to off
            {
                intake.setPower(0.0);
            }
        }

        if (within(intake.getPower(), INTAKE_POWER_EJECT, .01) && !gamepad1.right_bumper)
        {
            intake.setPower(0.0);
        }
        else if (gamepad1.right_bumper)
        {
            intake.setPower(INTAKE_POWER_EJECT);
        }
    }

    void dumpStats(boolean update)
    {
        telemetry.addLine("armPos" + arm.getCurrentPosition());
        telemetry.addLine("armTarget " + armTarget);
        telemetry.addLine("armPower " + arm.getPower());
        telemetry.addLine("intakeSpeed " + intake.getPower());

        if (update)
        {
            telemetry.update();
        }
    }

    abstract public void runOpMode();
}
