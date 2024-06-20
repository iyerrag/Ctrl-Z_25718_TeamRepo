package org.firstinspires.ftc.teamcode;

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
        chassis robot = new chassis(FL, FR, BL, BR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.throttleTranslateRadDeg(1, 90, 1, 6.3);
        robot.throttleTranslateRadDeg(1, 195, 1, 2.0);
        robot.tankTurn(-0.15, 1, 0.62);
        robot.translateXY(0, 0, 1);
        robot.throttleTranslateRadDeg(1, 90, 1, 4.52);
        robot.tankTurn(-0.15, 1, 0.8);
        robot.throttleTranslateRadDeg(1, 90, 1, 6.3);
        robot.tankTurn(-2, 1, 0.5);
        robot.translateXY(0, 0, 1);
        robot.throttleTranslateRadDeg(1, 115, 1, 6.0);
        robot.tankTurn(-0.15, 1, 0.62);







        // run until the end of the match (driver presses STOP)
        /*
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();


        }*/
    }
}