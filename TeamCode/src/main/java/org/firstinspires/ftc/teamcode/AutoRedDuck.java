package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(preselectTeleOp="TeleOpRed")
public class AutoRedDuck extends LinearOpMode
{
    private DcMotorEx carouselLeft = null;
    private DcMotorEx carouselRight = null;
    private DcMotorEx intake = null;
    private Servo dumper = null;
    private DcMotorEx arm = null;

    TeamElementDetermination determiner = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carouselLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselLeft.setDirection(DcMotor.Direction.FORWARD);
        carouselRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "carouselRight");
        carouselRight.setDirection(DcMotor.Direction.REVERSE);

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        arm = (DcMotorEx)hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dumper = (Servo)hardwareMap.get(Servo.class, "dumper");

        determiner = new TeamElementDetermination(hardwareMap, telemetry);
        determiner.result();

        drive.setPoseEstimate(FieldConstants.redDuckStartingPose);




        int armTarget = TeleOp.ARM_HIGH;
        Vector2d shippingHubPos = new Vector2d((-2.5 * 12), (0.8 * 12));
        double shippingHubHeading = Math.toRadians(0);
        TeamElementDetermination.BarcodePosition position = determiner.result();

        if (position == TeamElementDetermination.BarcodePosition.Left)
        {
            armTarget = TeleOp.ARM_HIGH;
            shippingHubPos = new Vector2d(FieldConstants.redShippingHubX - FieldConstants.armHighOffset, FieldConstants.redShippingHubY);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Center)
        {
            armTarget = TeleOp.ARM_MEDIUM;
            shippingHubPos = new Vector2d(FieldConstants.redShippingHubX - FieldConstants.armMedOffset, FieldConstants.redShippingHubY);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Right)
        {
            armTarget = TeleOp.ARM_LOW;
            shippingHubPos = new Vector2d(FieldConstants.redShippingHubX - FieldConstants.armLowOffset, FieldConstants.redShippingHubY);
        }

        final int armTargetFinal = armTarget;


        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(FieldConstants.redDuckStartingPose)
                .strafeTo(new Vector2d((-5 * 12), (-4.9 * 12) )) // red duck carousel
                .waitSeconds(4) // Wait for ducks to fall
                .lineToLinearHeading(new Pose2d((-5 * 12), (-2 * 12), 0)) // In line with shipping hub
                .build();

        // Start moving arm to target

        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(new Pose2d((-5 * 12), (-2 * 12), 0))
                .strafeTo(shippingHubPos) // Approach shipping hub
                .build();

        // Release block

        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(new Pose2d(shippingHubPos, shippingHubHeading))
                .strafeTo(new Vector2d((-6 * 12) + 8, (-3 * 12))) // Return to scoring square
                .build();

        // Move arm to neutral
        // Wait 4s


        waitForStart();




        carouselLeft.setPower(0.5);
        carouselRight.setPower(-0.5);

        dumper.setPosition(TeleOp.DUMPER_HOLD);

        drive.followTrajectorySequence(seq1);

        // Start moving arm to target
        arm.setTargetPosition(armTargetFinal);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(400);

        sleep(2000);

        drive.followTrajectorySequence(seq2);

        // Release block
        dumper.setPosition(TeleOp.DUMPER_RELEASE);
        sleep(2000);

        // Start moving arm to neutral
        arm.setTargetPosition(TeleOp.ARM_INTAKE);
        dumper.setPosition(TeleOp.DUMPER_OPEN);
        arm.setVelocity(400);
        intake.setPower(-0.1);
        sleep(4000); // Wait for arm to return

        drive.followTrajectorySequence(seq3); // Go to ending box, park completely

        carouselRight.setPower(0.0);
        carouselLeft.setPower(0.0);
    }
}
