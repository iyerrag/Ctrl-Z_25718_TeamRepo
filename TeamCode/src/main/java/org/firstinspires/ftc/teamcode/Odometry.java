//NOTE: EXTREMELY IMPORTANT: DEGREE TO RADIAN CONVERSIONS NOT REMOVED (7.13.24)


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometry {
    // Constant "c" is the width between parallel odometry wheels
    static final double c = 32;
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
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Note: All other instance variables default to 0.0
    }

    // Functions to determine the distance moved by each encoder
    public double measureLeftEncoderChange() {
        int currentPositionLeft = leftEncoder.getCurrentPosition();
        int difference = currentPositionLeft - previousPositionLeft;
        this.previousPositionLeft = currentPositionLeft;
        return difference * distancePerTick;
    }

    private double measureRightEncoderChange(){
        int currentPositionRight = rightEncoder.getCurrentPosition();
        int difference = currentPositionRight - previousPositionRight;
        this.previousPositionRight = currentPositionRight;
        return difference * distancePerTick;
    }

    private double measureFrontEncoderChange(){
        int currentPositionFront = frontEncoder.getCurrentPosition();
        int difference = currentPositionFront - previousPositionFront;
        this.previousPositionFront = currentPositionFront;
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
        dx = (1 - Math.cos((dxmax + dxmin)/(c))) * c * (0.5 - (dxmin/(dxmax + dxmin))) - (c * dym / (dxmax + dxmin)) * (Math.sin((dxmax + dxmin) / (c)));
        dy = Math.sin((dxmax + dxmin) / (c)) * c * (0.5 - (dxmin/(dxmax + dxmin)))-  (c * dym / (dxmax + dxmin)) * (1 - Math.cos((dxmax + dxmin) / (c)));
        dtheta = (dxmax + dxmin) / (c);

        // Sign Change on the Basis of Left/Right Turn
       if(dxright > 0 && dxleft < 0){
           dx *= -1;
           //dtheta is positive here
       }
       else{ //dxright < 0 && dxleft > 0
           dtheta *= -1;
           //dx is positive here
       }

       // Sign Change on the Basis of Pivot Direction [Forwards/Backwards]
       if(Math.abs(dxright) > Math.abs(dxleft) && dxright > 0){
           //Do nothing; dy is positive
       }
       else if(Math.abs(dxright) > Math.abs(dxleft) && dxright < 0){
           dy *= -1;
       }
       else if(Math.abs(dxright) < Math.abs(dxleft) && dxleft > 0){
           //Do nothing; dy is positive
       }
       else if(Math.abs(dxright) < Math.abs(dxleft) &&  dxleft < 0){
           dy *= -1;
       }
       else{
           //error; should never be true
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

        dx = c * ((dxmin / (dxmax - dxmin)) + 0.5) * (1 - Math.cos((dxmax - dxmin) / c)) + ((dym * c) / (dxmax - dxmin)) * Math.sin((dxmax - dxmin) / c);
        dy = c * ((dxmin / (dxmax - dxmin)) + 0.5) * (Math.sin((dxmax - dxmin) / c)) + ((dym * c) / (dxmax - dxmin)) * (1 - Math.cos((dxmax - dxmin) / c));
        dtheta = (dxmax - dxmin) / c;

        dx *= 2.33;

        if(Math.abs(dxright) > Math.abs(dxleft)){
            dx *= -1;
        }
        if(dxleft < 0 || dxright < 0){
            dy *= -1;
        }
        if(dxleft > dxright){
            dtheta *= -1;
        }
    }


    public void calculateDifferentials(){
        double dxleft = -1.0 * measureLeftEncoderChange();
        double dxright = -1.0 *  measureRightEncoderChange();
        double dym = measureFrontEncoderChange();

        if(dxleft == dxright){
            vectorDifferentials(dxleft, dym);
        }
       /*else if(dxleft * dxright < 0){
            swivelDifferentials(dxleft, dxright, dym);
        }*/
        else if(dxleft * dxright >= 0){
            skewedArcDifferentials(dxleft, dxright, dym);
        }
    }
    // Matrix Multiplication Function:

    // Function to multiply
    // two matrices A[][] and B[][]
    public static double[][] multiplyMatrix(double[][] A, double[][] B)
    {
        int i, j, k;

        int row1 = A.length;
        int col1 = A[0].length;
        int row2 = B.length;
        int col2 = B[0].length;

        // Check if multiplication is Possible
        if (row2 != col1) {

            return new double[][]{{}};
        }

        // Matrix to store the result
        // The product matrix will
        // be of size row1 x col2
        double[][] c = new double[row1][col2];

        // Multiply the two matrices
        for (i = 0; i < row1; i++) {
            for (j = 0; j < col2; j++) {
                for (k = 0; k < row2; k++)
                    c[i][j] += A[i][k] * B[k][j];
            }
        }

        return c;
    }

    // Convert Robot Differentials to Field Coordinates

    private void updateGlobalCoordinates(){

        //Define transformation matrix and robot differential vector
        double[][] differentialVector = new double[][]{{dx}, {dy}, {1}};
        double[][] transformationMatrix = new double[][]{{Math.cos(theta), -1 *  Math.sin(theta), x},
                                                        {Math.sin(theta), Math.cos(theta), y },
                                                        {0, 0, 1}  };

        //Linear transformation of robot differential vector to global field coordinates
        double[][] primes = multiplyMatrix(transformationMatrix, differentialVector);


        //Update global field coordinates
        x = primes[0][0];
        y = primes[1][0];
        theta = theta + dtheta;
    }

    // Function to Perform All Odometry Calculations; Use in chassis class methods' loops
    public void updateOdometry(){
        calculateDifferentials();
        updateGlobalCoordinates();
    }

    public double[] getDifferentials(){
        return new double[] {dx, dy, dtheta};
    }

    // Function to Return Current Field Positions as an Array
    public double[] getPosition(){
        return new double[]{x, y, theta};
    }



}
