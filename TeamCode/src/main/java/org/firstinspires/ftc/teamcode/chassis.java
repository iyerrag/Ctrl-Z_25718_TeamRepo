package org.firstinspires.ftc.teamcode;

import java.lang.*;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.*;

public class chassis{

    // Initialize Instance Variables:
    static private DcMotor fL;
    static private DcMotor fR;
    static private DcMotor bL;
    static private DcMotor bR;
    static private EulerianOdometry localizer;

    static private ElapsedTime timer;

    static final double rightBias = 1.0;
    static final double leftBias = 1.0;


    static final double maxA = 0.3;
    public chassis(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR) {
        // Define Timer Objects:
        timer = new ElapsedTime();
        localizer = new EulerianOdometry(0.0, 0.0, 0.0, bL, bR, fL);

        // Define Motor Objects for Chassis:
        fL = FL;
        fR = FR;
        bL = BL;
        bR = BR;

        bL.setDirection(DcMotor.Direction.REVERSE);

        bR.setDirection(DcMotor.Direction.FORWARD);

        fL.setDirection(DcMotor.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);

       // extender.setDirection(DcMotor.Direction.FORWARD);
    }

    private boolean eqWT(double val1, double val2, double e){
        return Math.abs(val1 - val2) <= e;
    }


    //translation at a constant speed given x and y components of power
    //postiive x is to the right and positive y is forward
    public void translateXY(double powX, double powY, double time) {
        timer.reset();
        while (timer.seconds() <= time) {
            //vector of the front right and bottom left wheels: a
            //vector of the bottom right and front left wheels: b
            double a = (powX+powY)*(Math.pow(2, -0.5));
            double b = (-powX+powY)*(Math.pow(2, -0.5));
            fL.setPower(a * leftBias);
            fR.setPower(b * rightBias);
            bL.setPower(b * leftBias);
            bR.setPower(a * rightBias);
            localizer.updateOdometry();
        }
    }

    public void translateRadDeg(double radius, double theta, double time) {
        //theta is in degrees (north is 90 deg and positive degrees are clockwise), radius is in units of powX and powY
        timer.reset();
        theta = theta; theta *= Math.PI/180;
        double powX = - radius * Math.cos(theta);
        double powY = radius * Math.sin(theta);
        while (timer.seconds() <= time) {
            //vector of the front right and bottom left wheels: a
            //vector of the bottom right and front left wheels: b
            double a = (powX+powY)*(Math.pow(2, -0.5));
            double b = (-powX+powY)*(Math.pow(2, -0.5));
            fL.setPower(a * leftBias);
            fR.setPower(b * rightBias);
            bL.setPower(b * leftBias);
            bR.setPower(a * rightBias);
            localizer.updateOdometry();
        }
    }

    public void throttleTranslateRadDeg(double radius, double theta, double s, double time) {
        //theta is in degrees (north is 90 deg and positive degrees are clockwise), radius is in units of powX and powY
        timer.reset();
        theta = theta; theta *= Math.PI/180;
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
            localizer.updateOdometry();
        }
    }

    public void tankTurn(double powLeft, double powRight, double time){
        timer.reset();
        while(timer.seconds() <= time){
            fL.setPower(powLeft * leftBias);
            bL.setPower(powLeft * leftBias);
            fR.setPower(powRight * rightBias);
            bR.setPower(powRight * rightBias);
            localizer.updateOdometry();
        }
    }

    public void localize(double x, double y, double theta){
        localizer = new EulerianOdometry(x, y, theta, bL, bR, fL);
    }

    public void toWaypoint(double x, double y, double theta, double toleranceDist, double toleranceAng, double Kp, double Ki, double Kd, double thetaWeight, double accelLimXY, double accelLimTheta){

        double Px = 0;
        double Ix = 0;
        double Dx = 0;
        double Py = 0;
        double Iy = 0;
        double Dy = 0;
        double Ptheta = 0;
        double Itheta = 0;
        double Dtheta = 0;
        double previousTime;
        double currentTime = timer.nanoseconds();
        double dt;
        double correctionX = 0;
        double correctionY = 0;
        double correctionTheta = 0;
        double previousCorrectionX;
        double previousCorrectionY;
        double previousCorrectionTheta;

        // Read current odometry position
        localizer.updateOdometry();
        double[] currentPos = localizer.getPosition();
        double currentX = currentPos[0];
        double currentY = currentPos[1];
        double currentTheta = currentPos[2];

        // PID control to waypoint
        while(!eqWT(currentX, x, toleranceDist) || !eqWT(currentY, y, toleranceDist) || !eqWT(currentTheta, theta, toleranceAng)){
            // Read current odometry position
            localizer.updateOdometry();
            currentPos = localizer.getPosition();
            currentX = currentPos[0];
            currentY = currentPos[1];
            currentTheta = currentPos[2];
            previousTime = currentTime;
            currentTime = timer.nanoseconds();
            dt = currentTime - previousTime;

            // Update Proportional, Integral, and Derivative Errors
            Px = currentX - x;
            Py = currentY - y;
            Ptheta = currentTheta - theta;

            Ix += Px;
            Iy += Py;
            Itheta += Ptheta;

            Dx = Px / dt;
            Dy = Py / dt;
            Dtheta = Ptheta / dt;

            // Calculate correction (multiply components by -1):
            previousCorrectionX = correctionX;
            previousCorrectionY = correctionY;
            previousCorrectionTheta = correctionTheta;

            correctionX = -1 * (Kp * Px + Ki * Ix + Kd * Dx);
            correctionY = -1 * (Kp * Py + Ki * Iy + Kd * Dy);
            correctionTheta = -1 * (Kp * Ptheta + Ki * Itheta + Kd * Dtheta);

            // Check if correction is within accelLim of previous correction to avoid slip
            if(!eqWT(correctionX, previousCorrectionX, accelLimXY)){
                if(correctionX > previousCorrectionX){
                    correctionX = previousCorrectionX + accelLimXY;
                }
                else{
                    correctionX = previousCorrectionX - accelLimXY;
                }
            }
            if(!eqWT(correctionY, previousCorrectionY, accelLimXY)){
                if(correctionY > previousCorrectionY){
                    correctionY = previousCorrectionY + accelLimXY;
                }
                else{
                    correctionY = previousCorrectionY - accelLimXY;
                }
            }
            if(!eqWT(correctionTheta, previousCorrectionTheta, accelLimTheta)){
                if(correctionTheta > previousCorrectionTheta){
                    correctionTheta = previousCorrectionTheta + accelLimTheta;
                }
                else{
                    correctionTheta = previousCorrectionTheta - accelLimTheta;
                }
            }

            // Actuate Correction
            double a = (correctionX+correctionY)*(Math.pow(2, -0.5));
            double b = (-correctionX+correctionY)*(Math.pow(2, -0.5));
            fL.setPower(a * leftBias * (1 - correctionTheta) * thetaWeight);
            fR.setPower(b * rightBias * (1 + correctionTheta) * thetaWeight);
            bL.setPower(b * leftBias * (1 - correctionTheta) * thetaWeight);
            bR.setPower(a * rightBias * (1 + correctionTheta) * thetaWeight);
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public DcMotor getMotor(String MotorName){
        if(MotorName.equals("fL")){
            return fL;
        }
        else if(MotorName.equals("fR")){
            return fR;
        }
        else if(MotorName.equals("bL")){
            return bL;
        }
        else{
            return bR;
        }
    }
}
