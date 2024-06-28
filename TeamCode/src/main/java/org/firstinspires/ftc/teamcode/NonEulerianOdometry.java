package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class NonEulerianOdometry {
    // Constant "c" is the width between parallel odometry wheels
    static final double c = 32.0;
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

    // IMU Reference
    private static robotIMU imu;

    // Angle Mode
    private static String angleMode;


    // Constructor for Setting up Odometry in chassis class
    public NonEulerianOdometry(double startingX, double startingY, double startingTheta, DcMotor left, DcMotor right, DcMotor front, robotIMU IMU, String thetaMode){
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

        imu = IMU;
        angleMode = thetaMode;

        // Note: All other instance variables default to 0.0
    }

    // Functions to determine the distance moved by each encoder
    private double measureLeftEncoderChange() {
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
        dtheta = 0.0;
    }

    private void skewedArcDifferentials(double dxleft, double dxright, double dym){
        //double s1 = dxleft;
        //double s2 = dxright;
        //double s3 = dym;

        //dx = (c* (s1/(s2-s1) + 1/2 ) + s3) * Math.cos((s2-s1)/c);
        //dy = (c* (s1/(s2-s1) + 1/2 ) + s3) * Math.sin((s2-s1)/c);

        dx = (((c * dxleft) / (dxright - dxleft)) + 0.5 * c + dym ) * (1 - Math.cos((dxright - dxleft) / c));
        dy = (((c * dxleft) / (dxright - dxleft)) + 0.5 * c + dym ) * (Math.sin((dxright - dxleft) / c));
        //dx = (c * (dxleft / (dxright - dxleft) + 0.5) + dym) * Math.cos((dxright - dxleft) / c);
        //dy = (c * (dxleft / (dxright - dxleft) + 0.5) + dym) * Math.sin((dxright - dxleft) / c);

        dtheta = (dxright - dxleft) / c;
    }



    public double[] calculateDifferentials(){
        double dxleft = -1.0 * measureLeftEncoderChange();
        double dxright = -1.0 *  measureRightEncoderChange();
        double dym = 1.0 * measureFrontEncoderChange();
        if(dxleft == dxright){
           vectorDifferentials(dxleft, dym);
       }
       else{
            skewedArcDifferentials(dxleft, dxright, dym);
        }
        //}

        return new double[]{dx, dy, dtheta};
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
        double[][] differentialVector = new double[][]{{dx}, {dy}, {1.0}};
        double[][] transformationMatrix = new double[][]{{Math.cos(theta), -1.0 *  Math.sin(theta), x},
                                                        {Math.sin(theta), Math.cos(theta), y },
                                                        {0.0, 0.0, 1.0}  };

        //Linear transformation of robot differential vector to global field coordinates
        double[][] primes = multiplyMatrix(transformationMatrix, differentialVector);


        //Update global field coordinates
        x = primes[0][0];
        y = primes[1][0];
        if(angleMode.equals("Encoder")){
            theta = theta + dtheta;
        }
        else{
            theta = imu.updateAngle();
        }

    }

    // Function to Perform All Odometry Calculations; Use in chassis class methods' loops
    public double[] updateOdometry(){
        double[] returnVal = calculateDifferentials();
        updateGlobalCoordinates();
        return returnVal;
    }

    public double[] getDifferentials(){
        return new double[] {dx, dy, dtheta};
    }

    // Function to Return Current Field Positions as an Array
    public double[] getPosition(){
        return new double[]{x, y, theta * 180 / Math.PI};
    }

}
