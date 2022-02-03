package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Constants.Alliance;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpRed", group="Linear OpMode")
public class TeleOpRed extends TeleOpBase
{
    @Override
    public void runOpMode()
    {
        runBaseCatched(Alliance.Red);
    }
}
