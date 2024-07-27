package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import java.lang.*;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class chassis{

    // Initialize Instance Variables:
    static private DcMotor fL;
    static private DcMotor fR;
    static private DcMotor bL;
    static private DcMotor bR;
    static private NonEulerianOdometry localizer;
    static private robotIMU imu;
    static private ElapsedTime timer;

    static final double rightBias = 1.0;
    static final double leftBias = 1.0;
    static private String angleMode;

    private double waypointToleranceDist, waypointToleranceAng, waypointKp, waypointKi, waypointKd, waypointKcx, waypointKcy, waypointKctheta, waypointThetaWeight, waypointAccelLimXY, waypointAccelLimTheta, waypointClamping;
    static final double maxA = 0.3;

    public chassis(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR, BHI260IMU IMU, String thetaMode) {
        // Define Timer Objects:
        timer = new ElapsedTime();
        imu = new robotIMU(IMU);
        angleMode = thetaMode;
        localizer = new NonEulerianOdometry(0.0, 0.0, 0.0, BL, BR, FL, imu, angleMode);

        // Define Motor Objects for Chassis:
        fL = FL;
        fR = FR;
        bL = BL;
        bR = BR;

        bL.setDirection(DcMotor.Direction.REVERSE);

        bR.setDirection(DcMotor.Direction.FORWARD);

        fL.setDirection(DcMotor.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);

        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        bR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));


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
        localizer = new NonEulerianOdometry(x, y, theta, bL, bR, fL, imu, angleMode);
    }

    public void waypointSettings(double toleranceDist, double toleranceAng, double Kp, double Ki, double Kd, double Kcx, double Kcy, double Kctheta, double thetaWeight, double accelLimXY, double accelLimTheta, double clamping){
        waypointToleranceDist = toleranceDist;
        waypointToleranceAng = toleranceAng * Math.PI / 180;
        waypointKp = Kp;
        waypointKi = Ki;
        waypointKd = Kd;
        waypointKcx = Kcx;
        waypointKcy = Kcy;
        waypointKctheta = Kctheta;
        waypointThetaWeight = thetaWeight;
        waypointAccelLimXY = accelLimXY;
        waypointAccelLimTheta = accelLimTheta;
        waypointClamping = clamping;
    }

    public double[] toWaypoint(double waypointTargetX, double waypointTargetY, double waypointTargetTheta, double timeout){
        waypointTargetTheta *= Math.PI / 180.0;
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
        double currentTime = timer.seconds();
        double startTime = timer.seconds();
        double dt;
        double globalCorrectionX = 0;
        double globalCorrectionY = 0;
        double globalCorrectionTheta = 0;
        double previousGlobalCorrectionX;
        double previousGlobalCorrectionY;
        double previousGlobalCorrectionTheta;
        double previousErrX;
        double previousErrY;
        double previousErrTheta;
        double Kcx = waypointKcx;
        double Kcy = waypointKcy;
        double Kctheta = waypointKctheta;
        double errX = 0;
        double errY = 0;
        double errTheta = 0;
        double correctionVectorMagnitude = 0.0;
        double correctionVectorAngle = 0.0;
        double localCorrectionX = 0.0;
        double localCorrectionY = 0.0;


        // Read current odometry position
        localizer.updateOdometry();
        double[] currentPos = localizer.getPosition();
        double currentX = currentPos[0];
        double currentY = currentPos[1];
        double currentTheta = currentPos[2];

        // PID control to waypoint
        while((!eqWT(currentX, waypointTargetX, waypointToleranceDist) || !eqWT(currentY, waypointTargetY, waypointToleranceDist) || !eqWT(currentTheta, waypointTargetTheta, waypointToleranceAng)) && ((timer.seconds() - startTime) < timeout)){
            // Read current odometry position
            localizer.updateOdometry();
            currentPos = localizer.getPosition();
            currentX = currentPos[0];
            currentY = currentPos[1];
            currentTheta = currentPos[2];
            previousTime = currentTime;
            currentTime = timer.seconds();
            dt = currentTime - previousTime;

            // Update Proportional, Integral, and Derivative Errors
            previousErrX = errX;
            previousErrY = errY;
            previousErrTheta = errTheta;

            errX = waypointTargetX - currentX;
            errY = waypointTargetY - currentY;
            errTheta = waypointTargetTheta - currentTheta;

            Px = errX;
            Py = errY;
            Ptheta = errTheta;

            if(Px > 0){
               Kcx = Math.abs(waypointKcx);
            }
            else if(Px == 0){
                Kcx = 0;
            }
            else{
                Kcx = -Math.abs(waypointKcx);
            }

            if(Py > 0){
                Kcy = Math.abs(waypointKcy);
            }
            else if(Py == 0){
                Kcy = 0;
            }
            else{
                Kcy = -Math.abs(waypointKcy);
            }

            if(Ptheta > 0){
                Kctheta = Math.abs(waypointKctheta);
            }
            else if(Ptheta == 0){
                Kctheta = 0;
            }
            else{
                Kctheta = -Math.abs(waypointKctheta);
            }

            Ix += dt * (errX + previousErrX) / 2.0;
            Iy += dt * (errY + previousErrY) / 2.0;
            Itheta += dt * (errTheta + previousErrTheta) / 2.0;

            if(Ix > Math.abs(waypointClamping)){
                Ix = Math.abs(waypointClamping);
            }
            else if(Ix < -1 * Math.abs(waypointClamping)){
                Ix = -1 * Math.abs(waypointClamping);
            }

            if(Iy > Math.abs(waypointClamping)){
                Iy = Math.abs(waypointClamping);
            }
            else if(Iy < - Math.abs(waypointClamping)){
                Iy = -1 * Math.abs(waypointClamping);
            }

            if(Itheta > Math.abs(waypointClamping * waypointThetaWeight)){
                Itheta = Math.abs(waypointClamping * waypointThetaWeight);
            }
            else if(Itheta < - Math.abs(waypointClamping * waypointThetaWeight)){
                Itheta = -1 * Math.abs(waypointClamping * waypointThetaWeight);
            }

            Dx = (errX - previousErrX) / dt;
            Dy = (errY - previousErrY) / dt;
            Dtheta = (errTheta - previousErrTheta) / dt;

            // Calculate correction (multiply components by -1):
            previousGlobalCorrectionX = globalCorrectionX;
            previousGlobalCorrectionY = globalCorrectionY;
            previousGlobalCorrectionTheta = globalCorrectionTheta;

            globalCorrectionX = (waypointKp * Px + waypointKi * Ix + waypointKd * Dx + Kcx);
            globalCorrectionY = (waypointKp * Py + waypointKi * Iy + waypointKd * Dy + Kcy);
            globalCorrectionTheta = (waypointKp * Ptheta + waypointKi * Itheta + waypointKd * Dtheta + Kctheta);

            localCorrectionY = globalCorrectionY * Math.cos(currentTheta) - globalCorrectionX * Math.sin(currentTheta);
            localCorrectionX = globalCorrectionY * Math.sin(currentTheta) + globalCorrectionX * Math.cos(currentTheta);

            localCorrectionX *= 1.25;
            localCorrectionY *= 0.9;

            //localCorrectionY = globalCorrectionX * Math.cos(-1.0 * currentTheta) - globalCorrectionY * Math.sin(-1.0 * currentTheta);
            //localCorrectionX = globalCorrectionX * Math.sin(-1.0 * currentTheta) + globalCorrectionY * Math.cos(currentTheta);

            //localCorrectionX = 0.6;
            //localCorrectionY = 0.6;

            /*// Check if correction is within accelLim of previous correction to avoid slip
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
            double a = (correctionX  + correctionY)*(Math.pow(2, -0.5));
            double b = (-correctionX + correctionY)*(Math.pow(2, -0.5));
            fL.setPower(a * leftBias - correctionTheta * waypointThetaWeight);
            fR.setPower(b * rightBias + correctionTheta * waypointThetaWeight);
            bL.setPower(b * leftBias - correctionTheta * waypointThetaWeight);
            bR.setPower(a * rightBias + correctionTheta * waypointThetaWeight);
            */
            // Actuate Correction
            double denominator = Math.max(Math.abs(localCorrectionX) + Math.abs(localCorrectionY) + Math.abs(globalCorrectionTheta * waypointThetaWeight), 1.0);
            double a = (localCorrectionX  + localCorrectionY);
            double b = (-localCorrectionX + localCorrectionY);
            fL.setPower((a * leftBias - globalCorrectionTheta * waypointThetaWeight) / denominator);
            fR.setPower((b * rightBias + globalCorrectionTheta * waypointThetaWeight) / denominator);
            bL.setPower((b * leftBias - globalCorrectionTheta * waypointThetaWeight) / denominator);
            bR.setPower((a * rightBias + globalCorrectionTheta * waypointThetaWeight) / denominator);

            //return new double[]{localCorrectionX, localCorrectionY, globalCorrectionX, globalCorrectionY, a, b};
       }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
        return new double[]{0.0, 0.0, 0.0, 0.0};
    }

    public void deflectTo(double deflectionXtarget, double deflectionYtarget, double deflectionThetatarget, double XYTol, double AngTol, double finalXtarget, double finalYtarget, double finalThetatarget, double timeout){
        bL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        bR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        fL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        fR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));
        double prevTolDist = waypointToleranceDist;
        double prevTolAng = waypointToleranceAng;
        double prevKp = waypointKp;
        double prevKi = waypointKi;
        double prevKd = waypointKd;
        double prevKcx = waypointKcx;
        double prevKcy = waypointKcy;
        double prevKctheta = waypointKctheta;
        double prevThetaWeight = waypointThetaWeight;
        double prevAccelLimXY = waypointAccelLimXY;
        double prevAccelLimTheta = waypointAccelLimTheta;
        double prevClamping = waypointClamping;
        waypointSettings(XYTol, AngTol,.027,0.0027, .0027, .00375, .00375, .00375, 15, .1, .1, .1);
        toWaypoint(deflectionXtarget, deflectionYtarget, deflectionThetatarget, timeout);
        waypointSettings(prevTolDist, prevTolAng, prevKp, prevKi, prevKd, prevKcx, prevKcy, prevKctheta, prevThetaWeight, prevAccelLimXY, prevAccelLimTheta, prevClamping);
        toWaypoint(finalXtarget, finalYtarget, finalThetatarget, timeout);
        bL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        bR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        fR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void gyroTurn(double powLeft, double powRight, double targetAngle){
        targetAngle *= Math.PI / 180.0;
        double[] pos = localizer.getPosition();
        if(targetAngle > pos[2]){
            while(pos[2] < targetAngle){
                fL.setPower(powLeft * leftBias);
                bL.setPower(powLeft * leftBias);
                fR.setPower(powRight * rightBias);
                bR.setPower(powRight * rightBias);
                localizer.updateOdometry();
                pos = localizer.getPosition();
            }
        }
        else if(targetAngle < pos[2]){
            while(pos[2] > targetAngle){
                fL.setPower(powLeft * leftBias);
                bL.setPower(powLeft * leftBias);
                fR.setPower(powRight * rightBias);
                bR.setPower(powRight * rightBias);
                localizer.updateOdometry();
                pos = localizer.getPosition();
            }
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

    public void updateOdometry(){

        localizer.updateOdometry();
    }

    public double[] getPosition(){
        return localizer.getPosition();
    }

    public double getAngle(){
        return imu.updateAngle()[0];
    }

}
