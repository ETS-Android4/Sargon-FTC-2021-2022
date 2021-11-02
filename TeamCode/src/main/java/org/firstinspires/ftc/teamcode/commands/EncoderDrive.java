package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.Robot;

public class EncoderDrive implements Command {
    Robot robot;

    int targetPosition;
    double maxTime;

    boolean success = false;

    public EncoderDrive(Robot robot, int targetPosition, double maxTime)
    {
        this.robot = robot;
        this.targetPosition = targetPosition;
        this.maxTime = maxTime;
    }

    public Boolean run()
    {
        robot.getDrive().encoderDrive(targetPosition);
        return success = false;
    }

    public void cont()
    {

    }

    public Boolean succeeded()
    {
        return success;
    }
}
