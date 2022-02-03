package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Constants.Alliance;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpBlue", group="Linear OpMode")
public class TeleOpBlue extends TeleOpBase
{
    @Override
    public void runOpMode()
    {
        runBaseCatched(Alliance.Blue);
    }
}
