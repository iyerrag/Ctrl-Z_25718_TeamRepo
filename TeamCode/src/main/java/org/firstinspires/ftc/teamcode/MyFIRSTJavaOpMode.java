package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.chassis;


@Autonomous

public class MyFIRSTJavaOpMode extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FL = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BackRight");
        BHI260IMU IMU = hardwareMap.get(BHI260IMU.class, "imu");
        chassis robot = new chassis(FL, FR, BL, BR, IMU);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.throttleTranslateRadDeg(1, 45, 1, 5);
        robot.toWaypoint(120, 120, 0, 0.2, 0.1,  .009,0.000, 0.0, 0.1, 0.3, .1, .1);







        // run until the end of the match (driver presses STOP)
        /*
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();


        }*/
    }
}