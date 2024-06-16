package org.firstinspires.ftc.teamcode;

import java.lang.*;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class chassis{

    // Initialize Instance Variables:
    static private DcMotor fL;
    static private DcMotor fR;
    static private DcMotor bL;
    static private DcMotor bR;
    static private DcMotor extender;

    static private ElapsedTime timer;

    static final double rightBias = 0.87;
    static final double leftBias = 1.0;

    static final double maxA = 0.3;
    public chassis(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR, DcMotor Extender) {
        // Define Timer Object:
        timer = new ElapsedTime();

        // Define Motor Objects for Chassis:
        fL = FL;
        fR = FR;
        bL = BL;
        bR = BR;
        extender = Extender;

        bL.setDirection(DcMotor.Direction.REVERSE);

        bR.setDirection(DcMotor.Direction.FORWARD);

        fL.setDirection(DcMotor.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);

        extender.setDirection(DcMotor.Direction.FORWARD);

    }

    //translation at a constant speed given x and y components of power
    //postiive x is to the right and positive y is forward
    public void translateXY(double powX, double powY, double time) {
        timer.reset();
        powX *= -1;
        while (timer.seconds() <= time) {
            //vector of the front right and bottom left wheels: a
            //vector of the bottom right and front left wheels: b
            double a = (powX+powY)*(Math.pow(2, -0.5));
            double b = (-powX+powY)*(Math.pow(2, -0.5));
            fL.setPower(b * leftBias);
            fR.setPower(a * rightBias);
            bL.setPower(a * leftBias);
            bR.setPower(b * rightBias);
        }
    }

    public void translateRadDeg(double radius, double theta, double time) {
        //theta is in degrees (north is 0 deg and positive degrees are clockwise), radius is in units of powX and powY
        timer.reset();
        theta = -theta + 90; theta *= Math.PI/180;
        double powX = - radius * Math.cos(theta);
        double powY = radius * Math.sin(theta);
        while (timer.seconds() <= time) {
            //vector of the front right and bottom left wheels: a
            //vector of the bottom right and front left wheels: b
            double a = (powX+powY)*(Math.pow(2, -0.5));
            double b = (-powX+powY)*(Math.pow(2, -0.5));
            fL.setPower(b * leftBias);
            fR.setPower(a * rightBias);
            bL.setPower(a * leftBias);
            bR.setPower(b * rightBias);
        }
    }

    public void throttleTranslateRadDeg(double radius, double theta, double s, double time) {
        //theta is in degrees (north is 0 deg and positive degrees are clockwise), radius is in units of powX and powY
        timer.reset();
        theta = -theta + 90; theta *= Math.PI/180;
        double powX = -radius * Math.cos(theta);
        double powY = radius * Math.sin(theta);
        double mp;
        while (timer.seconds() <= time) {
            mp = -1 * Math.abs(maxA * (2.0 * timer.seconds() - time * s ) / 4.0) - Math.abs(maxA * (2.0 * timer.seconds() - 2.0 * time + time * s) / 4.0) + (maxA * time / 2.0);
            //vector of the front right and bottom left wheels: a
            //vector of the bottom right and front left wheels: b
            double a = (powX+powY)*(Math.pow(2, -0.5));
            double b = (-powX+powY)*(Math.pow(2, -0.5));
            fL.setPower(mp * b * leftBias);
            fR.setPower(mp * a * rightBias);
            bL.setPower(mp * a * leftBias);
            bR.setPower(mp * b * rightBias);
        }
    }

    public void tankTurn(double powLeft, double powRight, double time){
        timer.reset();
        while(timer.seconds() <= time){
            fL.setPower(powLeft * leftBias);
            bL.setPower(powLeft * leftBias);
            fR.setPower(powRight * rightBias);
            bR.setPower(powRight * rightBias);
        }
    }

    public void extend(double time){
        timer.reset();
        while(timer.seconds() <= time){
            extender.setPower(1);
        }
        extender.setPower(0);
    }
}
