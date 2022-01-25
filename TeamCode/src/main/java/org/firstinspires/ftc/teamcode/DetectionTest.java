package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class DetectionTest extends LinearOpMode
{
    TeamElementDetermination determiner = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        determiner = new TeamElementDetermination(hardwareMap, telemetry);
        determiner.result();
    }
}
