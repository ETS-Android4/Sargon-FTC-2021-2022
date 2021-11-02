package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

public class Robot {
    ElapsedTime runtime = new ElapsedTime();
    Telemetry telm = null;

    MecanumDrive drive = null;
    DcMotor claw = null;

    WebcamName webcam1 = null;
    OpenGLMatrix location;

    Robot(Telemetry telm, HardwareMap hardwareMap, String[] motorNames, String camera1Name)
    {
        this.telm = telm;
        drive = new MecanumDrive(hardwareMap, motorNames);
        webcam1 = hardwareMap.get(WebcamName.class, camera1Name);

        location = OpenGLMatrix.identityMatrix();
    }

    public void start()
    {
        runtime.reset();
    }

    public MecanumDrive getDrive()
    {
        return drive;
    }

    public WebcamName getWebcam1Name()
    {
        return webcam1;
    }

    public OpenGLMatrix getLocation()
    {
        return location;
    }

    public void setLocation(OpenGLMatrix to)
    {
        location = to;
    }
}
