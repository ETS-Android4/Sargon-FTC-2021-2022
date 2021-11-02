package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private DcMotorEx driveFrontLeft = null;
    private DcMotorEx driveFrontRight = null;
    private DcMotorEx driveBackLeft = null;
    private DcMotorEx driveBackRight = null;

    MecanumDrive(HardwareMap hardwareMap, String[] motorNames)
    {
        driveFrontLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, motorNames[0]);
        driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        driveFrontRight = (DcMotorEx)hardwareMap.get(DcMotor.class, motorNames[1]);
        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, motorNames[2]);
        driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        driveBackRight = (DcMotorEx)hardwareMap.get(DcMotor.class, motorNames[3]);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void move(double direction, double power)
    {
        double x = power * Math.cos(direction);
        double y = power * Math.sin(direction);

        double drive = -y;
        double strafe = -x;

        double frontLeftPower = drive + strafe;
        double frontRightPower = -drive + strafe;
        double backLeftPower = drive - strafe;
        double backRightPower = -drive - strafe;

        driveFrontLeft.setPower(frontLeftPower);
        driveFrontRight.setPower(frontRightPower);
        driveBackLeft.setPower(backLeftPower);
        driveBackRight.setPower(backRightPower);
    }

    // Direction: Radians between -PI and PI
    // Power: Double between 0 and 1
    public void rotate(double direction, double power)
    {
        driveFrontLeft.setPower((direction / Math.PI) * power);
        driveFrontRight.setPower((direction / Math.PI) * power);
        driveBackLeft.setPower((direction / Math.PI) * power);
        driveBackRight.setPower((direction / Math.PI) * power);
    }

    public void gamepadMove(double joyX, double joyY, double triggerL, double triggerR) {
        double r = Math.hypot(joyX, joyY);
        double robotAngle = Math.atan2(joyY, joyX);
        double yaw = triggerR - triggerL;
        //double frontLeftPower = (r * Math.cos(robotAngle)) + yaw;
        //double frontRightPower = (r * Math.sin(robotAngle)) - yaw;
        //double backLeftPower = (r * Math.sin(robotAngle)) + yaw;
        //double backRightPower = (r * Math.cos(robotAngle)) - yaw;

        double drive = joyY;
        double strafe = -joyX;
        double twist = triggerL - triggerR;

        double frontLeftPower = drive + strafe + twist;
        double frontRightPower = -drive + strafe + twist;
        double backLeftPower = drive - strafe + twist;
        double backRightPower = -drive - strafe + twist;

        driveFrontLeft.setPower(frontLeftPower);
        driveFrontRight.setPower(frontRightPower);
        driveBackLeft.setPower(backLeftPower);
        driveBackRight.setPower(backRightPower);
    }

    public void encoderDrive(int distance)
    {
        driveFrontLeft.setTargetPosition(distance);
        driveFrontRight.setTargetPosition(distance);
        driveBackLeft.setTargetPosition(distance);
        driveBackRight.setTargetPosition(distance);

        driveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveFrontLeft.setPower(1);
        driveFrontRight.setPower(1);
        driveBackLeft.setPower(1);
        driveBackRight.setPower(1);
    }
}
