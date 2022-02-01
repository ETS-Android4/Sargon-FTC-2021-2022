package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DetectBlueThreshold;
import static org.firstinspires.ftc.teamcode.Constants.DetectRedThreshold;
import static org.firstinspires.ftc.teamcode.Constants.DetectXThreshold;
import static org.firstinspires.ftc.teamcode.Constants.alliance;
import static org.firstinspires.ftc.teamcode.Constants.within;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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





    public BarcodePosition result() throws InterruptedException
    {
        while (!pipeline.isReady) {
            sleep(100);
        }

        return pipeline.getAnalysis();
    }

    enum BarcodePosition
    {
        Left,
        Center,
        Right
    }

    public static class Pipeline extends OpenCvPipeline
    {
        Telemetry telemetry;

        volatile boolean isReady = false;

        private volatile BarcodePosition position = BarcodePosition.Left;

        Mat hierarchy = new Mat(); // Never used
        Mat channelOnly = new Mat();


        Pipeline(Telemetry _telemetry)
        {
            telemetry = _telemetry;
        }


        @Override
        public void init(Mat firstFrame)
        {
            processFrame(firstFrame);
        }




        @Override
        public Mat processFrame(Mat input)
        {
            int xSize = input.width();
            int ySize = input.height();

            org.opencv.core.Core.extractChannel(input, channelOnly, alliance == Constants.Alliance.Red ? 0 : 2);

            int threshold = alliance == Constants.Alliance.Red ? DetectRedThreshold : DetectBlueThreshold;

            Imgproc.threshold(channelOnly, channelOnly, threshold, 255, 0);

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(channelOnly, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


            List<Integer> sizes = new ArrayList<Integer>();

            for (int i = 0; i < contours.size(); i++)
            {
                Rect bounds = Imgproc.boundingRect((Mat) (contours.get(i)));


                sizes.add(bounds.width * bounds.height);
            }

            contours.removeIf(c ->
            {
                Rect bounds = Imgproc.boundingRect((Mat)(c));
                if ((bounds.width * bounds.height) < 200)
                {
                    return true;
                }

                if (bounds.y < (ySize / 4))
                {
                    return true;
                }

                return false;
            });

            //telemetry.addData(">", Arrays.toString(contours.toArray()));


            boolean centerFound = false;
            boolean rightFound = false;

            // Look for two pieces of tape
            for (int i = 0; i < contours.size(); i++)
            {
                Rect bounds = Imgproc.boundingRect((Mat)(contours.get(i)));

                if (bounds.x < (xSize / 2))
                {
                    centerFound = true;
                }
                else if (bounds.x > (xSize / 2))
                {
                    rightFound = true;
                }
            }

            if (centerFound && rightFound)
            {
                position = BarcodePosition.Left;
            }
            else if (centerFound && !rightFound)
            {
                position = BarcodePosition.Right;
            }
            else if (rightFound && !centerFound)
            {
                position = BarcodePosition.Center;
            }

            isReady = true;

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
