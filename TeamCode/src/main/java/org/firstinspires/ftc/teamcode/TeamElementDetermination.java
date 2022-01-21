package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamElementDetermination
{
    public OpenCvCamera frontWebcam = null;
    public TeamElementDetermination.Pipeline pipeline = null;

    public TeamElementDetermination(HardwareMap hardwareMap, Telemetry _telemetry)
    {
        WebcamName frontWebcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(frontWebcamName);
        pipeline = new Pipeline(_telemetry);
        frontWebcam.setPipeline(pipeline);
        frontWebcam.openCameraDevice();
        frontWebcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
    }





    public BarcodePosition result()
    {
        return pipeline.getAnalysis();
    }

    enum BarcodePosition
    {
        Unknown,
        Left,
        Center,
        Right
    }

    public static class Pipeline extends OpenCvPipeline
    {
        Telemetry telemetry;
        Pipeline(Telemetry _telemetry)
        {
            telemetry = _telemetry;
        }

        static final Scalar WHITE = new Scalar(255, 255, 255);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,62);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(112,62);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(250,62);
        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 114;

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
                Math.min(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, 320),
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1, region2, region3;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile BarcodePosition position = BarcodePosition.Left;

        @Override
        public void init(Mat firstFrame)
        {
            region1 = firstFrame.submat(new Rect(region1_pointA, region1_pointB));
            region2 = firstFrame.submat(new Rect(region2_pointA, region2_pointB));
            region3 = firstFrame.submat(new Rect(region3_pointA, region3_pointB));

            processFrame(firstFrame);
        }

        @Override
        public Mat processFrame(Mat input)
        {
            region1 = input.submat(new Rect(region1_pointA, region1_pointB));
            region2 = input.submat(new Rect(region2_pointA, region2_pointB));
            region3 = input.submat(new Rect(region3_pointA, region3_pointB));

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */

            double[] avgs1 = Core.mean(region1).val;
            double[] avgs2 = Core.mean(region2).val;
            double[] avgs3 = Core.mean(region3).val;

            // Average every RGB channel together since white is #FFFFFF
            double avg1 = mean(avgs1);
            double avg2 = mean(avgs2);
            double avg3 = mean(avgs3);

            /*
             * Find the max of the 3 averages
             */
            double maxOneTwo = Math.max(avg1, avg2);
            double max = Math.max(maxOneTwo, avg3);

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

            telemetry.addData(">", String.format("%f %f %f %s", avg1, avg2, avg3, position.toString()));
            telemetry.update();

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

        public static double mean(double[] arr) {
            double sum = 0;
            for (double value : arr) {
                sum += value;
            }
            return sum / (double)arr.length;
        }
    }
}
