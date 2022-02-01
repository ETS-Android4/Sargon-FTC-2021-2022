package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH;
import static org.firstinspires.ftc.teamcode.Constants.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_LOW;
import static org.firstinspires.ftc.teamcode.Constants.ARM_MEDIUM;
import static org.firstinspires.ftc.teamcode.Constants.ARM_VELOCITY_FAR;
import static org.firstinspires.ftc.teamcode.Constants.ARM_VELOCITY_HOLD;
import static org.firstinspires.ftc.teamcode.Constants.ARM_VELOCITY_NEAR;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_HOLD;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_OPEN;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_RELEASE;
import static org.firstinspires.ftc.teamcode.Constants.autoShippingOffset;
import static org.firstinspires.ftc.teamcode.Constants.within;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.exception.util.ExceptionContextProvider;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Constants.*;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous(preselectTeleOp="TeleOpBlue")
public class AutoBlueDuck extends LinearOpMode
{
    SampleMecanumDrive drive = null;
    private DcMotorEx carouselLeft = null;
    private DcMotorEx intake = null;
    private Servo dumper = null;
    private DcMotorEx arm = null;
    private int armTarget = 0;
    private ArmState armState = ArmState.AtZero;
    private boolean armResetting = false;
    private Timer timer = new Timer();
    private TeamElementDetermination determiner = null;

    public void updateArmState()
    {
        if (armTarget == 0 && within(arm.getCurrentPosition(), armTarget, 50))
        {
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
        else if (armTarget == 0 && within(arm.getCurrentPosition(), armTarget, 200))
        {
            arm.setVelocity(ARM_VELOCITY_HOLD);
        }
        else if (armTarget == 0)
        {
            arm.setVelocity(ARM_VELOCITY_FAR);
        }

        if (armTarget != 0 && within(arm.getCurrentPosition(), armTarget, 150))
        {
            armState = ArmState.NearLevel;
            arm.setVelocity(ARM_VELOCITY_NEAR);
        }
        else if (armTarget != 0 && within(arm.getCurrentPosition(), armTarget, 50))
        {
            armState = ArmState.AtLevel;
            arm.setVelocity(ARM_VELOCITY_HOLD);
        }
        else if (armTarget != 0)
        {
            arm.setVelocity(ARM_VELOCITY_FAR);
        }
    }

    @Override
    public void runOpMode()
    {
        try {
            runCaught();
        }
        catch (Exception e)
        {
            telemetry.addData("!", "OPMODE CRASHED: " + e.getMessage());
        }
    }

    public void runCaught() throws InterruptedException
    {
        drive = new SampleMecanumDrive(hardwareMap);

        carouselLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselLeft.setDirection(DcMotor.Direction.FORWARD);

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        arm = (DcMotorEx)hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dumper = (Servo)hardwareMap.get(Servo.class, "dumper");

        determiner = new TeamElementDetermination(hardwareMap, telemetry);

        drive.setPoseEstimate(Constants.blueDuckStartingPose);

        waitForStart();


        armTarget = ARM_HIGH;
        Vector2d shippingHubPos = drive.getPoseEstimate().vec();
        double shippingHubHeading = Math.toRadians(0);
        TeamElementDetermination.BarcodePosition position = determiner.result();

        if (position == TeamElementDetermination.BarcodePosition.Right)
        {
            armTarget = ARM_HIGH;
            shippingHubPos = new Vector2d(Constants.blueShippingHubX - Constants.armHighOffset, Constants.blueShippingHubY);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Center)
        {
            armTarget = ARM_MEDIUM;
            shippingHubPos = new Vector2d(Constants.blueShippingHubX - Constants.armMedOffset, Constants.blueShippingHubY);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Left)
        {
            armTarget = ARM_LOW;
            shippingHubPos = new Vector2d(Constants.blueShippingHubX - Constants.armLowOffset, Constants.blueShippingHubY);
        }

        final int armTargetFinal = armTarget; // Required for value to be used inside a lambda


        carouselLeft.setPower(-0.5);
        dumper.setPosition(DUMPER_HOLD); // Prevents block escape during lifting

        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d((-4.95 * 12), (4.9 * 12) )) // Blue duck carousel
                .waitSeconds(4) // Wait for ducks to fall
                .lineToLinearHeading(new Pose2d((-5 * 12), (2 * 12), 0)) // In line with shipping hub
                .build();
        drive.followTrajectorySequenceAsync(seq1);

        while (true)
        {
            drive.update();

            updateArmState();

            //todo: catch hitting duck wheel
            if (!drive.isBusy())
            {
                break;
            }
        }

        // Start moving arm to target
        arm.setTargetPosition(armTargetFinal);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(400);
        armState = ArmState.ToLevel;

        sleep(5000);


        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(shippingHubPos, shippingHubHeading)) // Approach shipping hub
                .build();
        drive.followTrajectorySequenceAsync(seq2);

        while (true)
        {
            drive.update();

            updateArmState();

            if (!drive.isBusy())
            {
                break;
            }
        }

        // Release block
        dumper.setPosition(DUMPER_RELEASE);
        sleep(2000);


        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d((-6 * 12) + 8, (3 * 12))) // Return to scoring square
                .build();
        drive.followTrajectorySequenceAsync(seq3); // Go to ending box, park completely

        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                dumper.setPosition(DUMPER_OPEN);
            }
        }, 500);



        // Start moving arm to neutral
        arm.setTargetPosition(ARM_INTAKE);
        arm.setVelocity(400);
        intake.setPower(-0.1);
        armState = ArmState.ToZero;
        sleep(4000); // Wait for arm to return

        while (true)
        {
            drive.update();

            updateArmState();

            if (!drive.isBusy())
            {
                break;
            }
        }

        carouselLeft.setPower(0.0);

        sleep(1000);


        Constants.setRobotCurrentPose(drive.getPoseEstimate());
    }
}
