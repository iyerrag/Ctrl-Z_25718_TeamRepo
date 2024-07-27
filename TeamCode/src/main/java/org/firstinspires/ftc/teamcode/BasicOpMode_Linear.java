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
import com.qualcomm.robotcore.hardware.Servo;
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
        claw gripper = new claw(hardwareMap.get(Servo.class, "LeftTalon"), hardwareMap.get(Servo.class, "RightTalon"), hardwareMap.get(DcMotor.class, "Extender"));

        double[] position = new double[3];
        double[] differentials = new double[3];
        double primes;
        double gyroAngle = 0.0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double[] ab = {0, 0, 0, 0, 0, 0};

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
            if(gamepad1.a){
                /*robot.waypointSettings(1, 1,  .0135,0.000, 0.0, 0.25, 0.1, 0.1, 0.33, .1, .1);
                robot.toWaypoint(-120, 0, 45);
                robot.toWaypoint(-60, 120, 0);
                robot.toWaypoint(-120, 240, 15);
                robot.toWaypoint(0, 240, -15);
                robot.toWaypoint(-60, 120, -45);
                robot.waypointSettings(.1, 1,  .0135,0.000, 0.0, 0.15, 0.1, 0.1, 0.33, .1, .1);
                robot.toWaypoint(0, 0, 0);*/


                //robot.waypointSettings(1.5, 1.5,.027,0.0027, .0027, .0075, .00375, .00375, 15, .1, .1, .1);
                robot.waypointSettings(1, 1.5,.027,0.0054, .0108, 0, 0, 0, 12, .1, .1, 14);
                robot.toWaypoint(120, 120,0, 3);
                robot.toWaypoint(0,120,90,3);
                robot.toWaypoint(0,0,0,3);
                //robot.toWaypoint(0, 120, 0, 10);
                //robot.toWaypoint(0, 0,0,10);
                /*robot.deflectTo(-60,60,0, 10,10, -60,110, 0, 2.5);
                robot.toWaypoint(-60,0,0,1.5);
                robot.toWaypoint(-140, 0, -90, 2.5);
                robot.deflectTo(-140, 120, -90, 5,5,-120, 120, -90, 3.5);
                robot.deflectTo(-140, 120, -90, 5, 5, -140, 0, -90, 3.5);
                robot.toWaypoint(0,0,0, 3.5);
                robot.deflectTo(0,240,0, 10,10, -110, 240, 180, 5);
                robot.toWaypoint(-110, 180, 180, 1.5);
                robot.deflectTo(-110, 240, 180, 20, 10, 0, 240, 0, 3);
                robot.toWaypoint(0,0,0, 2.5);
                robot.deflectTo(0, 170, 90, 5,5,-60, 170,90, 2.5);
                robot.deflectTo(0, 170, 90, 5,5,0,0,0, 2.5);*/



                //robot.toWaypoint(0, 60, 0, 1);
               // robot.deflectTo(-60, 100, 0, 20, 10, 0, 180, 90, 3);
                //robot.deflectTo(-60, 240, 90, 20, 10, -120, 180, 90, 3);
               // robot.deflectTo(-60, 120, 90, 20, 10, -120, 60, 90, 3);
                //robot.deflectTo(-60, 0, 90, 20, 10, 0, 0, 0, 3);





                /*robot.toWaypoint(0, 240, 0);
                robot.toWaypoint(-120, 240, 0);
                robot.toWaypoint(-120, 0, 0);
                robot.toWaypoint(0,0,0);
                robot.toWaypoint(0, 240, 0);
                robot.toWaypoint(0, 240, 90);
                robot.toWaypoint(-120, 240, 90);
                robot.toWaypoint(-120, 240, 180);
                robot.toWaypoint(-120, 0, 180);
                robot.toWaypoint(-120, 0, 270);
                robot.toWaypoint(0, 0,270);
                robot.toWaypoint(0,0,0);
                robot.toWaypoint(0, 240, 90);
                robot.toWaypoint(-120, 240, 180);
                robot.toWaypoint(-120, 0, 270);
                robot.toWaypoint(0, 0, 0);
                //robot.waypointSettings(1, 1,  0.027,0.0027, .0027, .00375, .00375, .00375, 15, .1, .1, .1);
                /*robot.toWaypoint(0, 240, 45);
                robot.toWaypoint(0, 0, 0);
                robot.toWaypoint(-120, 0, 0);
                robot.waypointSettings(1, 1,  0.027,0, 0.027 * 0.05, 0, 0, 0.1, 1, .1, .1);
                robot.toWaypoint(-120, 0, 180);
                robot.toWaypoint(-120, 0, 0);
                robot.toWaypoint(0,0,0);
                robot.gyroTurn(-1, 1, 90);
                robot.gyroTurn(1, -1, 0);*/



                /*robot.toWaypoint(-60, 240, 45, 2);
                robot.toWaypoint(-60, 240, 0, 2);
                robot.toWaypoint(0,0,0,2);
                robot.toWaypoint(-60, 120, 0,2);
                robot.toWaypoint(-120, 240, 15,2);
                robot.toWaypoint(0, 240, -15,2);
                robot.toWaypoint(-60, 120, -45,2);
                robot.toWaypoint(0, 0, 0,2);
                robot.toWaypoint(0, 240, 0,2);
                robot.toWaypoint(-60, 0, -30,2);
                robot.toWaypoint(-120, 240, 30,2);
                robot.toWaypoint(-120, 0, -30,2);
                robot.toWaypoint(-60, 240, 30,2);
                robot.waypointSettings(1.5, 1.5,.027,0.0027, .0027, .00375, .00375, .00375, 15, .1, .1, .1);
                robot.toWaypoint(0, 0, 0, 3);*/
            }
            else if(gamepad1.b){
                gripper.open();
            }
            else if(gamepad1.x){
                gripper.liftTo(0,0);
                telemetry.addData("LiftPow: ", gripper.getLiftPos());
            }
            else if(gamepad1.y){
                gripper.liftTo(1000, 0);
                telemetry.addData("LiftPos: ", gripper.getLiftPos());
            }
            else if(gamepad1.right_bumper){
                gripper.close();
            }
            else{
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

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());


            telemetry.addData("X:", "" + Math.round(position[0] * 100.0) / 100.0);
            telemetry.addData("Y:", "" + Math.round(position[1] * 100.0) / 100.0);
            telemetry.addData("Theta:", "" + (Math.round(position[2] * 100.0) / 100.0) * 180.0 / Math.PI);
            telemetry.addData("Gyro Angle:", "" + gyroAngle * 180.0 / Math.PI);
            /*telemetry.addData("globalCorrectionX: ", "" + ab[0]);
            telemetry.addData("globalCorrectionY: ", "" + ab[1]);
            telemetry.addData("correctionX: ", "" + ab[2]);
            telemetry.addData("correctionY: ", "" + ab[3]);
            telemetry.addData("correctionVectorMagnitude: ", "" + ab[4]);
            telemetry.addData("correctionVectorAngle: ", "" + ab[5]);*/

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