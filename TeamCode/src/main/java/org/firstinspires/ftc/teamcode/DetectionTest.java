package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class DetectionTest extends LinearOpMode
{
    TeamElementDetermination determiner = null;

    public static Constants.Alliance color = Constants.Alliance.Red;

    @Override
    public void runOpMode() throws InterruptedException
    {
        determiner = new TeamElementDetermination(hardwareMap, telemetry);

        Constants.alliance = color;


        waitForStart();

        while (!isStopRequested())
        {
            telemetry.addData(">", determiner.result().toString());
            telemetry.update();

            sleep(100);
        }



    }
}
