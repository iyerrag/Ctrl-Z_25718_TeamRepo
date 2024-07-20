package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


/*
     * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
     * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
     * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     *
     * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
     * It includes all the skeletal structure that all linear OpModes contain.
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode liTt
     */


    @TeleOp
    public class ControlTest extends LinearOpMode {

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor fL;
        private DcMotor fR;
        private DcMotor bL;
        private DcMotor bR;

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            fL = hardwareMap.get(DcMotor.class, "FrontLeft");
            fR = hardwareMap.get(DcMotor.class, "FrontRight");
            bL = hardwareMap.get(DcMotor.class, "BackLeft");
            bR = hardwareMap.get(DcMotor.class, "BackRight");
            BHI260IMU IMU = hardwareMap.get(BHI260IMU.class, "imu");

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

            chassis robot = new chassis(fL, fR, bL, bR, IMU, "IMU");

            double[] data = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double[] pos = new double[]{0.0, 0.0, 0.0};

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();
            //0.027 * 0.05
            robot.waypointSettings(1, 1, 0.027, 0,0 , 0.1, 0.1, 0.1, .33, .1, .1, 1);
            Thread.sleep(10000);
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                data = robot.toWaypoint(-60, 240, 0, 10);
                pos = robot.getPosition();
                fL.setPower(0);
                fR.setPower(0);
                bL.setPower(0);
                bR.setPower(0);
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Local Correction Vector: ", "<" + (double) Math.round(data[0] * 100) / 100.0 + "," + (double) Math.round(data[1] * 100) / 100.0 + ">");
                telemetry.addData("Global Correction Vector", "<" + (double) Math.round(data[2] * 100) / 100.0 + "," + (double) Math.round(data[3] * 100) / 100.0 + ">");
                telemetry.addData("a: ", data[4]);
                telemetry.addData("b: ", data[5]);
                telemetry.update();
                Thread.sleep(3000);
            }

        }
    }
