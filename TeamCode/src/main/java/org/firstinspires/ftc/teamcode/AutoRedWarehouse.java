package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ARM_HIGH;
import static org.firstinspires.ftc.teamcode.Constants.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.ARM_LOW;
import static org.firstinspires.ftc.teamcode.Constants.ARM_MEDIUM;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_HOLD;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_OPEN;
import static org.firstinspires.ftc.teamcode.Constants.DUMPER_RELEASE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Constants.*;

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

        drive.setPoseEstimate(Constants.redWarehouseStartingPose);


        int armTarget = ARM_HIGH;
        Vector2d shippingHubPos = Constants.redShippingHubPos;
        double shippingHubHeading = Math.toRadians(135);
        TeamElementDetermination.BarcodePosition position = determiner.result();

        if (position == TeamElementDetermination.BarcodePosition.Left)
        {
            armTarget = ARM_HIGH;
            shippingHubPos = new Vector2d(Constants.redShippingHubX + Constants.armHighSquareOffset, Constants.redShippingHubY - Constants.armHighSquareOffset);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Center)
        {
            armTarget = ARM_MEDIUM;
            shippingHubPos = new Vector2d(Constants.redShippingHubX + Constants.armMedSquareOffset, Constants.redShippingHubY - Constants.armHighSquareOffset);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Right)
        {
            armTarget = ARM_LOW;
            shippingHubPos = new Vector2d(Constants.redShippingHubX + Constants.armLowSquareOffset, Constants.redShippingHubY - Constants.armHighSquareOffset);
        }

        final int armTargetFinal = ARM_HIGH;

        // Extend arm
        // Wait 2s

        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(Constants.redWarehouseStartingPose)
                .lineToLinearHeading(new Pose2d(shippingHubPos, shippingHubHeading)) // In line with shipping hub
                .build();

        // Place piece
        // Wait 2s
        // Return arm
        // Wait 2s

        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(new Pose2d(shippingHubPos, shippingHubHeading))
                .lineToLinearHeading(new Pose2d(Constants.redWarehouseStartingPoseX, Constants.redWarehouseStartingPoseY + 10, 0)) // In line with shipping hub
                .strafeRight(10)
                .forward(2.5 * 12)
                .build();

        waitForStart();

        dumper.setPosition(DUMPER_HOLD);

        // Start moving arm to target
        arm.setTargetPosition(armTargetFinal);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(400);
        sleep(2000);

        drive.followTrajectorySequence(seq1);

        dumper.setPosition(DUMPER_RELEASE);
        sleep(2000);

        // Start moving arm to neutral
        arm.setTargetPosition(ARM_INTAKE);
        dumper.setPosition(DUMPER_OPEN);
        arm.setVelocity(400);
        intake.setPower(-0.1);
        sleep(4000); // Wait for arm to return

        drive.followTrajectorySequence(seq2);

        intake.setPower(0.0);
        dumper.setPosition(DUMPER_OPEN);
    }
}
