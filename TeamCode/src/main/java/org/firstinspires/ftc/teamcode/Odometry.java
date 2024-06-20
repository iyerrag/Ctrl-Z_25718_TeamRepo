package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometry {
    // Constant "c" is the width between parallel odometry wheels
    static final double c = 10.0;
    // Constant "r" is the radius of the odometry wheels in centimeters
    static final double r = 2.4;
    // Constant "ticksPerRev" is the number of ticks per odometry wheel revolution
    static final double ticksPerRev = 2000.0;
    // Constant "distancePerTick" is the distance moved per odometry wheel tick [autocalculated]
    static final double distancePerTick = 2 * Math.PI * r / ticksPerRev;

    // Field Position Variables; Note x and y are in unit cm while theta is in radians
    private double x;
    private double y;
    private double theta;

    // Robot Position Differentials
    private double dx;
    private double dy;
    private double dtheta;

    // Previous Tick Counts
    private int previousPositionLeft;
    private int previousPositionRight;
    private int previousPositionFront;

    // Motor Encoders
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor frontEncoder;


    // Constructor for Setting up Odometry in chassis class
    public Odometry(double startingX, double startingY, double startingTheta, DcMotor left, DcMotor right, DcMotor front){
        x = startingX;
        y = startingY;
        theta = startingTheta;
        leftEncoder = left;
        rightEncoder = right;
        frontEncoder = front;
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Note: All other instance variables default to 0.0
    }

    // Functions to determine the distance moved by each encoder
    public double measureLeftEncoderChange(){
        int currentPositionLeft = leftEncoder.getCurrentPosition();
        int difference = currentPositionLeft - previousPositionLeft;
        previousPositionLeft = currentPositionLeft;
        return difference * distancePerTick;
    }

    private double measureRightEncoderChange(){
        int currentPositionRight = rightEncoder.getCurrentPosition();
        int difference = currentPositionRight - previousPositionRight;
        previousPositionRight = currentPositionRight;
        return difference * distancePerTick;
    }

    private double measureFrontEncoderChange(){
        int currentPositionFront = frontEncoder.getCurrentPosition();
        int difference = currentPositionFront - previousPositionFront;
        previousPositionFront = currentPositionFront;
        return difference * distancePerTick;
    }

    // Functions to calculate robot differentials

    private void vectorDifferentials(double dxm, double dym){
        dx = dym;
        dy = dxm;
        dtheta = 0;
    }

    private void swivelDifferentials(double dxleft, double dxright, double dym){

        double dxmax;
        double dxmin;

        if(Math.abs(dxleft) < Math.abs(dxright)){
            dxmax = Math.abs(dxright);
            dxmin = Math.abs(dxleft);
        }
        else{
            dxmax = Math.abs(dxleft);
            dxmin = Math.abs(dxright);
        }
        dx = (1 - Math.cos((dxmax + dxmin)/(2 * Math.PI * c))) * c * (0.5 - (dxmin/(dxmax + dxmin))) - (2 * Math.PI * c * dym / (dxmax + dxmin)) * (Math.sin((dxmax + dxmin) / (2 * Math.PI * c)));
        dy = Math.sin((dxmax + dxmin) / (2 * Math.PI * c)) * c * (0.5 - (dxmin/(dxmax + dxmin)))-  (2 * Math.PI * c * dym / (dxmax + dxmin)) * (1 - Math.cos((dxmax + dxmin) / (2 * Math.PI * c)));
        dtheta = (dxmax + dxmin) / (2 * c * Math.PI);

        if((Math.abs(dxleft) < Math.abs(dxright) && dxright > 0) || (Math.abs(dxleft) > Math.abs(dxright) && dxleft > 0)){
            if(Math.abs(dxleft) < Math.abs(dxright)){
                dx *= -1;
            }
            else{
                dtheta *= -1;
            }
        }
        else{
            dy *= -1;
            if(Math.abs(dxleft) < Math.abs(dxright)){
                dx *= -1;
                dtheta *= -1;
            }
        }

    }

    private void skewedArcDifferentials(double dxleft, double dxright, double dym){
        double dxmax;
        double dxmin;

        if(Math.abs(dxleft) < Math.abs(dxright)){
            dxmax = Math.abs(dxright);
            dxmin = Math.abs(dxleft);
        }
        else{
            dxmax = Math.abs(dxleft);
            dxmin = Math.abs(dxright);
        }

        dx = c * ((dxmin / (dxmax - dxmin)) + 0.5) * (1 - Math.cos((dxmax - dxmin) / c)) - ((dym * c) / (dxmax - dxmin)) * Math.sin((dxmax - dxmin) / c);
        dy = c * ((dxmin / (dxmax - dxmin)) + 0.5) * (Math.sin((dxmax - dxmin) / c)) - ((dym * c) / (dxmax - dxmin)) * (1 - Math.cos((dxmax - dxmin) / c));
        dtheta = (dxmax - dxmin) / c;

        if(Math.abs(dxright) > Math.abs(dxleft)){
            dx *= -1;
        }
        if(Math.abs(dxright + dxleft) < 0){
            dy *= -1;
        }
        if(dxleft > dxright){
            dtheta *= -1;
        }
    }


    private void calculateDifferentials(){
        double dxleft = -1.0 * measureLeftEncoderChange();
        double dxright = -1.0 *  measureRightEncoderChange();
        double dym = -1.0 * measureFrontEncoderChange();

        if(dxleft == dxright){
            vectorDifferentials(dxleft, dym);
        }
        else if(dxleft * dxright < 0){
            swivelDifferentials(dxleft, dxright, dym);
        }
        else{
            skewedArcDifferentials(dxleft, dxright, dym);
        }
    }

    // Convert Robot Differentials to Field Coordinates

    private void updateGlobalCoordinates(){
        double xprime = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2)) * Math.cos(Math.atan(dy / dx) + theta) + x;
        double yprime = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2)) * Math.sin(Math.atan(dy / dx) + theta) + y;
        double thetaprime = theta + dtheta;

        x = xprime;
        y = yprime;
        theta = thetaprime;
    }

    // Function to Perform All Odometry Calculations; Use in chassis class methods' loops
    public void updateOdometry(){
        calculateDifferentials();
        updateGlobalCoordinates();
    }

    // Function to Return Current Field Positions as an Array
    public double[] getPosition(){
        return new double[]{x, y, theta};
    }



}
