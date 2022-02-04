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
import static org.firstinspires.ftc.teamcode.Constants.ARM_POWER_MANUAL;
import static org.firstinspires.ftc.teamcode.Constants.ARM_VELOCITY_FAR;
import static org.firstinspires.ftc.teamcode.Constants.ARM_VELOCITY_HOLD;
import static org.firstinspires.ftc.teamcode.Constants.ARM_VELOCITY_NEAR;
import static org.firstinspires.ftc.teamcode.Constants.CAROUSEL_SPEED_CAP;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_HOLD;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_OPEN;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_RELEASE;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER_EJECT;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_POWER_SCALAR;
import static org.firstinspires.ftc.teamcode.Constants.within;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
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
    // right stick y: arm target height
    // shoulders: rotate
    // d-pad: arm position:
        // Up: spin carousel
        // Down: Ready for intake
        // Left: None
        // Right: High level
    // a: dumper hold
    // b: dumper release
    // x: dumper open for intake
    // y: reverse drive controls
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
        2: None
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
public abstract class TeleOpBase extends LinearOpMode
{
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }


    ElapsedTime runtime = new ElapsedTime();
    Timer timer = new Timer();

    Telemetry driveTelemetry;
    Telemetry hubTelemetry;

    GamepadEx p1;
    GamepadEx p2;

    private SampleMecanumDriveCancelable drive;
    private boolean useReversed = true;
    private Mode currentMode = Mode.DRIVER_CONTROL;

    // Used to spin duck discs
    private DcMotorEx carouselLeft;

    private DcMotorEx intake;

    private DcMotorEx arm;
    private int armTarget = 0;
    private ArmState armState = ArmState.AtZero;
    private boolean armResetting = false;
    private boolean armUseVelocity = true;

    private Servo dumper;

    Alliance alliance = Alliance.Blue;


    public void initMotors()
    {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (Constants.poseSet)
        {
            drive.setPoseEstimate(Constants.robotCurrentPose);
        }
        else // If pose was not specifically set by auto, just try to get the right side of the field
        {
            drive.setPoseEstimate(alliance == Alliance.Blue ? Constants.blueDuckStartingPose :
                    Constants.redDuckStartingPose);
        }

        carouselLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselLeft.setDirection(DcMotor.Direction.FORWARD);
        carouselLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        arm = (DcMotorEx)hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        armTarget = 0;
        armState = ArmState.AtZero;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ARM_VELOCITY_NEAR);

        PIDFCoefficients armPidf = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        dumper = (Servo)hardwareMap.get(Servo.class, "dumper");
    }

    void setArmTarget(int target)
    {
        armTarget = target;

        arm.setTargetPosition(target);

        if (arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (armTarget != 0)
        {
            intake.setPower(0.0);
            armState = ArmState.ToLevel;
            armResetting = false;
        }
        else
        {
            armState = ArmState.ToLevel;
        }
    }

    void runArm()
    {
        if (p1.wasJustPressed(Button.RIGHT_STICK_BUTTON))
        {
            armUseVelocity = !armUseVelocity;

            if (armUseVelocity)
            {
                arm.setTargetPosition(armTarget);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setVelocity(ARM_VELOCITY_NEAR);
            }
            else
            {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        if (armUseVelocity) {

            if (p1.wasJustPressed(Button.DPAD_DOWN)) {
                setArmTarget(0);

                // Spin intake to allow arm to move down, and automatically stop it after a couple seconds
                intake.setPower(-0.1);

                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        if (within(intake.getPower(), -0.1, 0.05)) {
                            intake.setPower(0.0);
                        }
                    }
                }, 3 * 1000);
            } else if (p1.wasJustPressed(Button.DPAD_RIGHT)) {
                setArmTarget(ARM_HIGH);
            } else {
                if (armTarget == 0 && within(arm.getCurrentPosition(), armTarget, 50)) {
                    armState = ArmState.AtZero;

                    if (!armResetting) {
                        armResetting = true;
                        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        arm.setPower(0.0);

                        timer.schedule(new TimerTask() {
                            @Override
                            public void run() {
                                if (armResetting) {
                                    if (arm.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                    }
                                    armResetting = false;
                                }
                            }
                        }, 500);
                    }
                }
            }


            if (armTarget != 0 && within(arm.getCurrentPosition(), armTarget, 150)) {
                armState = ArmState.NearLevel;
            } else if (armTarget != 0 && within(arm.getCurrentPosition(), armTarget, 50)) {
                armState = ArmState.AtLevel;
            }

            // Reset zero point on arm
            if (p2.wasJustPressed(Button.A)) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        else
        {
            arm.setPower(p1.getRightY());
        }
    }


    void runDumper()
    {
        if (gamepad2.left_stick_y != 0.0)
        {
            dumper.setPosition(dumper.getPosition() - (gamepad2.left_stick_y / 50));
        }
        else if (p1.wasJustPressed(Button.A))
        {
            dumper.setPosition(DUMPER_HOLD);
        }
        else if (p1.wasJustPressed(Button.X))
        {
            dumper.setPosition(DUMPER_OPEN);
        }
        else if (p1.wasJustPressed(Button.B))
        {
            dumper.setPosition(DUMPER_RELEASE);

            /*timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    dumper.setPosition(DUMPER_OPEN);
                }
            }, 500);*/
        }
    }

    void runDrive()
    {
        if (p1.wasJustPressed(Button.Y))
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
                if (p1.wasJustPressed(Button.Y))
                {
                    useReversed = !useReversed;
                }

                if (useReversed)
                {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    gamepad1.left_trigger - gamepad1.right_trigger
                            )
                    );
                }
                else
                {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    gamepad1.left_stick_y,
                                    gamepad1.left_stick_x,
                                    gamepad1.left_trigger - gamepad1.right_trigger
                            )
                    );
                }
                break;
            case AUTOMATIC_CONTROL:
                // If x is pressed, we break out of the automatic following
                if (false) {
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
        carouselLeft.setPower(gamepad1.dpad_up ? CAROUSEL_SPEED_CAP : 0.0);
                //(gamepad1.y ? CAROUSEL_SPEED_CAP : 0.0));
        //carouselLeft.setVel
    }

    void runIntake()
    {
        if (p1.wasJustPressed(Button.LEFT_BUMPER))
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

        if (within(intake.getPower(), INTAKE_POWER_EJECT, .05) && !gamepad1.right_bumper)
        {
            intake.setPower(0.0);
        }
        else if (gamepad1.right_bumper)
        {
            intake.setPower(INTAKE_POWER_EJECT);
        }
    }

    public static boolean logExtra = true;

    void dumpStats(boolean update)
    {
        if (logExtra)
        {
            telemetry.addData("armPos", arm.getCurrentPosition());
            telemetry.addData("armTarget ", armTarget);
            telemetry.addData("armVelocity ", arm.getVelocity());
            telemetry.addData("intakeSpeed ", intake.getPower());
            telemetry.addData("intakeVelocity ", intake.getVelocity());
            telemetry.addData("carouselVelocity ", carouselLeft.getVelocity());
            telemetry.addData("wheelVelocity ", drive.getWheelVelocities().get(3));

            telemetry.addLine("accel " + drive.getAccel());


            PIDFCoefficients armPidf = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addLine(String.format("Arm %f %f %f %f", armPidf.p, armPidf.i, armPidf.d, armPidf.f));



            if (true)
            {
                telemetry.update();
            }
        }
    }

    abstract public void runOpMode();


    public void runBaseCatched(Alliance alliance)
    {
        try {
            runBase(alliance);
        }
        catch (Exception e)
        {

        }
    }

    public void runBase(Alliance alliance)
    {
        alliance = Constants.Alliance.Blue;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initMotors();

        p1 = new GamepadEx(gamepad1);
        p2 = new GamepadEx(gamepad2);

        //gamepad1.setJoystickDeadzone(0.1);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested())
        {
            p1.readButtons();
            p2.readButtons();

            runArm();

            runDumper();

            runDriveSmart();

            runCarousel();

            runIntake();

            dumpStats(true);

        }
    }
}
