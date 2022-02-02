package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DetectBlueThreshold;
import static org.firstinspires.ftc.teamcode.Constants.DetectRedThreshold;
import static org.firstinspires.ftc.teamcode.Constants.alliance;
import static org.firstinspires.ftc.teamcode.Constants.telemetry;
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName frontWebcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(frontWebcamName, cameraMonitorViewId);
        pipeline = new Pipeline(_telemetry);
        frontWebcam.setPipeline(pipeline);
        frontWebcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        frontWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                frontWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("!", "FAILURE TO OPEN CAMERA!");
                telemetry.update();

                pipeline.position = Constants.autoHeightDefault;
                pipeline.isReady = true;
            }
        });
    }





    public BarcodePosition result() throws InterruptedException
    {
        while (!pipeline.isReady)
        {
            sleep(100);
        }

        return pipeline.position;
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

        public volatile boolean isReady = false;
        public volatile BarcodePosition position = BarcodePosition.Left;

        Mat hierarchy = new Mat(); // Never used
        //ArrayList<Mat> allChannels = new ArrayList<>(3);;

        boolean limitContours = false;


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
        public void onViewportTapped()
        {
            limitContours = !limitContours;
        }

        Mat channelOnly = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {
            int xSize = channelOnly.width();
            int ySize = channelOnly.height();

            org.opencv.core.Core.extractChannel(input, channelOnly, alliance == Constants.Alliance.Red ? 0 : 2);

            //Core.split(channelOnly, allChannels); // Mat src, ArrayList<Mat> dst
            //channelOnly = allChannels.get(alliance == Constants.Alliance.Red ? 0 : 2);
            //channelOnly = allChannels.get(limitContours ? 0 : 2);


            int threshold = alliance == Constants.Alliance.Red ? DetectRedThreshold : DetectBlueThreshold;

            Imgproc.threshold(channelOnly, channelOnly, threshold, 255, 0);

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(channelOnly, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<MatOfPoint> allContours = new ArrayList<>(contours);


            List<Integer> sizes = new ArrayList<Integer>();

            for (int i = 0; i < contours.size(); i++)
            {
                Rect bounds = Imgproc.boundingRect((Mat) (contours.get(i)));


                sizes.add(bounds.width * bounds.height);
            }

            contours.removeIf(c ->
            {
                Rect bounds = Imgproc.boundingRect((Mat)(c));
                if ((bounds.width * bounds.height) < 600)
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

                if (bounds.x < Constants.DetectCenterXRightThreshold)
                {
                    centerFound = true;
                }
                else if (bounds.x > Constants.DetectRightXLeftThreshold)
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


            Imgproc.drawContours(channelOnly, limitContours ? contours : allContours, -1, new Scalar(127,127,127), 2);

            Imgproc.line(channelOnly, new Point(Constants.DetectCenterXRightThreshold, 0),
                    new Point(Constants.DetectCenterXRightThreshold, channelOnly.height()), new Scalar(127,127,127));
            Imgproc.line(channelOnly, new Point(0, Constants.DectectTopThreshold),
                    new Point(channelOnly.width(), Constants.DectectTopThreshold), new Scalar(127,127,127));

            for (MatOfPoint i : allContours)
            {
                i.release();
            }



            return channelOnly;
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
