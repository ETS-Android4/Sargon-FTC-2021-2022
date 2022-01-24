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
public class AutoRedWarehouse extends LinearOpMode
{
    private DcMotorEx intake = null;
    private Servo dumper = null;
    private DcMotorEx arm = null;

    TeamElementDetermination determiner = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        arm = (DcMotorEx)hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dumper = (Servo)hardwareMap.get(Servo.class, "dumper");

        determiner = new TeamElementDetermination(hardwareMap, telemetry);
        determiner.result();

        drive.setPoseEstimate(FieldConstants.redWarehouseStartingPose);


        int armTarget = TeleOp.ARM_HIGH;
        Vector2d shippingHubPos = FieldConstants.redShippingHubPos;
        double shippingHubHeading = Math.toRadians(135);
        TeamElementDetermination.BarcodePosition position = determiner.result();

        if (position == TeamElementDetermination.BarcodePosition.Left)
        {
            armTarget = TeleOp.ARM_HIGH;
            shippingHubPos = new Vector2d(FieldConstants.redShippingHubX + FieldConstants.armHighSquareOffset, FieldConstants.redShippingHubY - FieldConstants.armHighSquareOffset);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Center)
        {
            armTarget = TeleOp.ARM_MEDIUM;
            shippingHubPos = new Vector2d(FieldConstants.redShippingHubX + FieldConstants.armMedSquareOffset, FieldConstants.redShippingHubY - FieldConstants.armHighSquareOffset);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Right)
        {
            armTarget = TeleOp.ARM_LOW;
            shippingHubPos = new Vector2d(FieldConstants.redShippingHubX + FieldConstants.armLowSquareOffset, FieldConstants.redShippingHubY - FieldConstants.armHighSquareOffset);
        }

        final int armTargetFinal = TeleOp.ARM_HIGH;

        // Extend arm
        // Wait 2s

        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(FieldConstants.redWarehouseStartingPose)
                .lineToLinearHeading(new Pose2d(shippingHubPos, shippingHubHeading)) // In line with shipping hub
                .build();

        // Place piece
        // Wait 2s
        // Return arm
        // Wait 2s

        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(new Pose2d(shippingHubPos, shippingHubHeading))
                .lineToLinearHeading(new Pose2d(FieldConstants.redWarehouseStartingPoseX, FieldConstants.redWarehouseStartingPoseY + 10, 0)) // In line with shipping hub
                .strafeRight(10)
                .forward(2.5 * 12)
                .build();

        waitForStart();

        dumper.setPosition(TeleOp.DUMPER_HOLD);

        // Start moving arm to target
        arm.setTargetPosition(armTargetFinal);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(400);
        sleep(2000);

        drive.followTrajectorySequence(seq1);

        dumper.setPosition(TeleOp.DUMPER_RELEASE);
        sleep(2000);

        // Start moving arm to neutral
        arm.setTargetPosition(TeleOp.ARM_INTAKE);
        dumper.setPosition(TeleOp.DUMPER_OPEN);
        arm.setVelocity(400);
        intake.setPower(-0.1);
        sleep(4000); // Wait for arm to return

        drive.followTrajectorySequence(seq2);

        intake.setPower(0.0);
        dumper.setPosition(TeleOp.DUMPER_OPEN);
    }
}
