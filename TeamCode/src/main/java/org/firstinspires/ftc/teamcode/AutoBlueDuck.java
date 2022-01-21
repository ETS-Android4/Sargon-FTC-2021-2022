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
import org.firstinspires.ftc.teamcode.TeleOp;
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

    public static double startingX = -3 * 12;
    public static double startingY = (6 * 12) + 8.375;
    public static double startingHeading = 0;

    private OpenCvCamera frontWebcam = null;
    private TeamElementDeterminationPipeline pipeline = null;



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

        WebcamName frontWebcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(frontWebcamName);
        pipeline = new TeamElementDeterminationPipeline(telemetry);
        frontWebcam.setPipeline(pipeline);
        frontWebcam.openCameraDevice();


        Pose2d startPose = new Pose2d(startingX, startingY, startingHeading);
        drive.setPoseEstimate(startPose);

        waitForStart();





        int armTarget = TeleOp.ARM_HIGH;
        Vector2d shippingHubPos = new Vector2d((-2.5 * 12), (0.8 * 12));
        double shippingHubHeading = Math.toRadians(45);
        if (pipeline.position == TeamElementDeterminationPipeline.BarcodePosition.Left)
        {
            armTarget = TeleOp.ARM_HIGH;
        }
        else if (pipeline.position == TeamElementDeterminationPipeline.BarcodePosition.Center)
        {
            armTarget = TeleOp.ARM_MEDIUM;
        }
        else if (pipeline.position == TeamElementDeterminationPipeline.BarcodePosition.Right)
        {
            armTarget = TeleOp.ARM_LOW;
        }

        final int armTargetFinal = armTarget;


        TrajectorySequence seq = drive.trajectorySequenceBuilder(startPose)
                .forward(2)
                .strafeTo(new Vector2d((-4.9 * 12), (4.9 * 12) )) // Blue duck carousel
                .waitSeconds(2) // Wait for ducks to fall
                .strafeTo(new Vector2d((-5 * 12), (0.5 * 12) ))
                .addDisplacementMarker(() ->
                {
                    // Start moving arm to target
                    arm.setTargetPosition(armTargetFinal);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);
                })
                .lineToLinearHeading(new Pose2d(shippingHubPos, Math.toRadians(45))) // Go to team shipping hub
                .addDisplacementMarker(() ->
                {
                    dumper.setPosition(TeleOp.DUMPER_RELEASE);
                })
                .waitSeconds(2)
                .addDisplacementMarker(() ->
                {
                    // Start moving arm to target
                    arm.setTargetPosition(TeleOp.ARM_INTAKE);
                    dumper.setPosition(TeleOp.DUMPER_OPEN);
                    arm.setPower(0.5);
                })
                .lineToLinearHeading(new Pose2d(new Vector2d((-5 * 12), (2.9 * 12)),  Math.toRadians(0)))
                .build();







        carouselLeft.setPower(0.5);
        carouselRight.setPower(0.5);

        //arm.setTargetPosition(-740);
        //arm.setDirection(DcMotorSimple.Direction.REVERSE);
        ///arm.setPower(-0.5);

        telemetry.addData("%", "Barcode Position: " + findObject().toString());
        telemetry.update();


        carouselRight.setPower(0.5);
        dumper.setPosition(TeleOp.DUMPER_HOLD);

        drive.followTrajectorySequence(seq);

        carouselRight.setPower(0.0);



    }

    public static class TeamElementDeterminationPipeline extends OpenCvPipeline
    {
        Telemetry telemetry;
        TeamElementDeterminationPipeline(Telemetry _telemetry)
        {
            telemetry = _telemetry;
        }

        enum BarcodePosition
        {
            Unknown,
            Left,
            Center,
            Right
        }

        static final Scalar WHITE = new Scalar(255, 255, 255);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(20,20);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(106,0);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(213,0);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile BarcodePosition position = BarcodePosition.Left;

        @Override
        public void init(Mat firstFrame)
        {
            region1_Cb = firstFrame.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = firstFrame.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = firstFrame.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            region1_Cb = input.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = input.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = input.submat(new Rect(region3_pointA, region3_pointB));

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */

            double[] avgs1 = Core.mean(region1_Cb).val;
            double[] avgs2 = Core.mean(region2_Cb).val;
            double[] avgs3 = Core.mean(region3_Cb).val;

            //avgs1.


            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            telemetry.addData(">", String.format("%f %f %f", avg1, avg2, avg3));
            telemetry.update();

            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                position = BarcodePosition.Left; // Record our analysis
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = BarcodePosition.Center; // Record our analysis
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = BarcodePosition.Right; // Record our analysis
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public BarcodePosition getAnalysis()
        {
            return position;
        }
    }

    private TeamElementDeterminationPipeline.BarcodePosition findObject()
    {


        frontWebcam.openCameraDevice();

        frontWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        //frontWebcam.getFrameBitmap();

        SimpleBlobDetector_Params params = new SimpleBlobDetector_Params();
        params.set_filterByColor(true);

        SimpleBlobDetector detector = SimpleBlobDetector.create(params);
        //detector.detectAndCompute();

        return TeamElementDeterminationPipeline.BarcodePosition.Unknown;
    }
}
