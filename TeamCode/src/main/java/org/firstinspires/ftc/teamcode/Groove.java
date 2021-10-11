package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileDescriptor;
import java.util.List;

import android.app.Application;
import android.media.MediaPlayer;
import android.media.AudioManager;
import android.media.AudioDeviceInfo;
import java.io.File;
import java.io.IOException;
import android.content.Context;
import java.lang.Thread;
import java.lang.StackTraceElement;
import java.util.Stack;
import android.net.Uri;


@TeleOp(name = "Groove", group = "Linear Opmode")
public class Groove extends LinearOpMode {
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

        intake.setVelocity(toggleX ? -37000 : 0, AngleUnit.RADIANS);
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


        if (gamepad1.a) {
            prevA = true;
        } else if (!gamepad1.a && prevA) {
            toggleA = !toggleA;
            prevA = false;
        }
        claw.setPosition(gamepad1.right_stick_y);

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




        /*Thread currentThread = Thread.currentThread();
        StackTraceElement[] trace = currentThread.getStackTrace();
        for (StackTraceElement i : trace)
        {
            telemetry.addData(">", i.toString());
            telemetry.update();
        }*/




        //Activity controllerActivity = org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
        /*Context context = AppUtil.getDefContext();
        AudioManager audioManager = context.getSystemService(AudioManager.class);
        
        AudioDeviceInfo speakerDevice = null;
        AudioDeviceInfo[] devices = audioManager.getDevices(AudioManager.GET_DEVICES_OUTPUTS);

        for (AudioDeviceInfo i : devices)
        {
            telemetry.addData(">", i.toString());
            telemetry.update();
        }*/




        //MediaPlayer player = null; // = new MediaPlayer();//this, music);
        //player.setAudioStreamType(AudioManager.STREAM_MUSIC);


        //Uri uri = Uri.fromFile(new File(AppUtil.ROBOT_DATA_DIR.getPath() + "/amogus.mp3"));
        //MediaPlayer player = MediaPlayer.create(AppUtil.getDefContext(), uri);


        /*try {

            //player.setDataSource(AppUtil.ROBOT_DATA_DIR.getPath() + "/amogus.mp3");
        }
        catch (IOException e) {
            telemetry.addData(">", "Failed to load file");
            telemetry.update();
        }*/
        //player.prepare();
        //player.start();







        while (opModeIsActive()) {
            runMecanum(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.left_trigger, gamepad1.right_trigger);
            arm.setVelocity(gamepad1.right_stick_y);

            //claw.setPosition((gamepad1.a) ? 1 : 0.5);

            updateToggles();
        }

        telemetry.addData(">", "Ended");
        telemetry.update();
    }

}
