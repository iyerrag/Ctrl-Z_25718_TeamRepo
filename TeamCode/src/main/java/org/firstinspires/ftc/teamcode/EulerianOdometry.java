package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class EulerianOdometry {
    // Constant "c" is the width between parallel odometry wheels
    static final double c = 29.6;
    // Constant "r" is the radius of the odometry wheels in centimeters
    static final double r = 2.4;
    // Constant "ticksPerRev" is the number of ticks per odometry wheel revolution
    static final double ticksPerRev = 2000.0;
    // Constant "distancePerTick" is the distance moved per odometry wheel tick [autocalculated]
    static final double distancePerTick = 2 * Math.PI * r / ticksPerRev;
    // Constant "forwardOffset" is the distance in cm between the rear odometry wheels and the front odometry wheel
    static final double forwardOffset = 9.8;

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

    // IMU Instance Variables
    static private robotIMU imu;
    static private String angleMode;
    static private double imuAngleReading;


    // Constructor for Setting up Odometry in chassis class
    public EulerianOdometry(double startingX, double startingY, double startingTheta, DcMotor left, DcMotor right, DcMotor front,  robotIMU IMU, String thetaMode){
        x = startingX;
        y = startingY;
        theta = startingTheta;
        leftEncoder = left;
        rightEncoder = right;
        frontEncoder = front;

        imu = IMU;
        angleMode = thetaMode;
        imuAngleReading = 0.0;

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


    public void calculateDifferentials(){
        double dxleft = -1.0 * measureLeftEncoderChange();
        double dxright = -1.0 *  measureRightEncoderChange();
        double dym = measureFrontEncoderChange();
        double[] measuredIMUAngle = imu.updateAngle();
        dy = (dxleft + dxright) / 2.0;
        dtheta = (dxright - dxleft) / c;
        if(angleMode.equals("Encoder")){
            dx = dym - forwardOffset * dtheta;
        }
        else{
            dx = dym - forwardOffset * measuredIMUAngle[1];
        }

        imuAngleReading = measuredIMUAngle[0];

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
        if(angleMode.equals("Encoder")){
            theta += dtheta;
        }
        else{
            theta = imuAngleReading;
        }
        x = primes[0][0];
        y = primes[1][0];

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
