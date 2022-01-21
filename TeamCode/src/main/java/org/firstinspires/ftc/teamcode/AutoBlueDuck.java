package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.TeamElementDetermination;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

@Autonomous
@Config
public class AutoBlueDuck extends LinearOpMode
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

        drive.setPoseEstimate(FieldConstants.blueDuckStartingPose);

        waitForStart();





        int armTarget = TeleOp.ARM_HIGH;
        Vector2d shippingHubPos = new Vector2d((-2.5 * 12), (0.8 * 12));
        double shippingHubHeading = Math.toRadians(45);
        TeamElementDetermination.BarcodePosition position = determiner.result();

        if (position == TeamElementDetermination.BarcodePosition.Left)
        {
            armTarget = TeleOp.ARM_HIGH;
            shippingHubPos = new Vector2d(FieldConstants.blueShippingHubPos - FieldConstants.armHighOffset,
                    FieldConstants.blueShippingHubPos + FieldConstants.armHighOffset);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Center)
        {
            armTarget = TeleOp.ARM_MEDIUM;
            shippingHubPos = new Vector2d(FieldConstants.blueShippingHubPos - FieldConstants.armMedOffset,
                    FieldConstants.blueShippingHubPos + FieldConstants.armMedOffset);
        }
        else if (position == TeamElementDetermination.BarcodePosition.Right)
        {
            armTarget = TeleOp.ARM_LOW;
            shippingHubPos = new Vector2d(FieldConstants.blueShippingHubPos - FieldConstants.armLowOffset,
                    FieldConstants.blueShippingHubPos + FieldConstants.armLowOffset);
        }
        else
        {
            telemetry.addData("!", "INVALID BARCODE");
            telemetry.update();
        }

        final int armTargetFinal = armTarget;


        TrajectorySequence seq = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d((-4.9 * 12), (4.9 * 12) )) // Blue duck carousel
                .waitSeconds(2) // Wait for ducks to fall
                .lineToLinearHeading(new Pose2d((-5 * 12), (2 * 12), 0)) // In line with shipping hub
                .addDisplacementMarker(() ->
                {
                    // Start moving arm to target
                    arm.setTargetPosition(armTargetFinal);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);
                })
                .strafeTo(new Vector2d(FieldConstants.blueShippingHubX - FieldConstants.armHighOffset, FieldConstants.blueShippingHubY)) // Approach shipping hub
                .addDisplacementMarker(() ->
                {
                    dumper.setPosition(TeleOp.DUMPER_RELEASE);
                })
                .waitSeconds(2)
                .addDisplacementMarker(() ->
                {
                    // Start moving arm to neutral
                    arm.setTargetPosition(TeleOp.ARM_INTAKE);
                    dumper.setPosition(TeleOp.DUMPER_OPEN);
                    arm.setPower(0.5);
                })
                .strafeTo(new Vector2d((-5 * 12), (2 * 12))) // Return to scoring square
                .strafeTo(new Vector2d((-5 * 12), (2.9 * 12)))
                .build();




        while(!isStopRequested())
        {
            sleep(50);
        }


        carouselLeft.setPower(0.5);
        carouselRight.setPower(0.5);


        carouselRight.setPower(0.5);
        dumper.setPosition(TeleOp.DUMPER_HOLD);

        drive.followTrajectorySequence(seq);

        carouselRight.setPower(0.0);
        dumper.setPosition(TeleOp.DUMPER_OPEN);



    }
}
