/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    @Override
    public void runOpMode() {
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

        double[] position = new double[3];
        double[] differentials = new double[3];
        double primes;
        double gyroAngle = 0.0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
            if(!gamepad1.a) {
                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double powX = gamepad1.right_stick_x;
                double powY = -gamepad1.right_stick_y;
                double addLeft = 0.5 * gamepad1.left_stick_x;
                double addRight = -0.5 * gamepad1.left_stick_x;


                // Send calculated power to wheels
                double a = (powX + powY) * (Math.pow(2, -0.5));
                double b = (-powX + powY) * (Math.pow(2, -0.5));
                fL.setPower(a + addLeft);
                fR.setPower(b + addRight);
                bL.setPower(b + addLeft);
                bR.setPower(a + addRight);

                robot.updateOdometry();
                position = robot.getPosition();
                gyroAngle = Math.round(robot.getAngle() * 100.0) / 100.0;
                /*double A[][] = { { 1, 1, 1 },
                        { 2, 2, 2 },
                        { 3, 3, 3 },
                        { 4, 4, 4 } };

                double B[][] = { { 1, 1, 1, 1 },
                        { 2, 2, 2, 2 },
                        { 3, 3, 3, 3 } };

                double[][] c = Odometry.multiplyMatrix(A, B);*/

               // telemetry.addData("dx: ", "" + temp[0]);

                //telemetry.addData("dy: ", "" + temp[1]);

               // telemetry.addData("dTheta: ", "" + temp[2]);
            }
            else{
                robot.toWaypoint(-120, 0, 30, 1, 1,  .0135,0.000, 0.0, 0.1, 0.33, .1, .1);
                robot.toWaypoint(-60, 120, 0, 1, 1,  .0135,0.00, 0.0, 0.1, .33, .1, .1);
                robot.toWaypoint(-120, 240, -30, 1, 1,  .0135,0.00, 0.0, 0.1, .33, .1, .1);
                robot.toWaypoint(0, 240, 0, 1, 1,  .0135,0.00, 0.0, 0.1, .33, .1, .1);
                robot.toWaypoint(-60, 120, 30, 1, 1,  .0135,0.00, 0.0, 0.1, .33, .1, .1);
                robot.toWaypoint(0, 0, 0, .05, 1,  .0135,0.00, 0.0, 0.1, .33, .1, .1);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());


            telemetry.addData("X:", "" + Math.round(position[0] * 100.0) / 100.0);
            telemetry.addData("Y:", "" + Math.round(position[1] * 100.0) / 100.0);
            telemetry.addData("Theta:", "" + Math.round(position[2] * 100.0) / 100.0);
            telemetry.addData("Gyro Angle:", "" + gyroAngle);


            /*telemetry.addData("dx:", "" + differentials[0]);
            telemetry.addData("dy:", "" + differentials[1]);
            telemetry.addData("dtheta:", "" + differentials[2]);
            telemetry.addData("leftCount':", "" + primes);
            telemetry.addData("y':", "" + primes[1]);
            telemetry.addData("theta':", "" + primes[2]);
            telemetry.addData("", "" + c[0][0] + "" + c[0][1] + "" + c[0][2] + "" + c[0][3]);
            telemetry.addData("", "" + c[1][0] + "" + c[1][1] + "" + c[1][2] + "" + c[1][3]);
            telemetry.addData("", "" + c[2][0] + "" + c[2][1] + "" + c[2][2] + "" + c[2][3]);
            telemetry.addData("", "" + c[3][0] + "" + c[3][1] + "" + c[3][2] + "" + c[3][3]);*/

            telemetry.update();
        }
    }
}
