// NOTE: EXTREMELY IMPORTANT: IMU-BASED CONTROL NOT COMPLETELY IMPLEMENTED
// NOTE: EXTREMELY IMPORTANT: IMU DEGREES TO RADIAN CONVERSIONS NOT ELIMINATED (7.13.24)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class NonEulerianOdometry {
    // Constant "c" is the width between parallel odometry wheels
    static final double c = 30;
    // Constant "r" is the radius of the odometry wheels in centimeters
    static final double r = 2.375;
    // Constant "ticksPerRev" is the number of ticks per odometry wheel revolution
    static final double ticksPerRev = 2000.0;
    // Constant "distancePerTick" is the distance moved per odometry wheel tick [autocalculated]
    static final double distancePerTick = 2 * Math.PI * r / ticksPerRev;
    // Forward Offset
    static final double forwardOffset = 10;

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

    private static double imuGlobalAngle;
    private static double imuDeltaAngle;


    // Constructor for Setting up Odometry in chassis class
    public NonEulerianOdometry(double startingX, double startingY, double startingTheta, DcMotor left, DcMotor right, DcMotor front, robotIMU IMU, String thetaMode){
        x = startingX;
        y = startingY;
        theta = startingTheta * Math.PI / 180;
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
        imuGlobalAngle = 0.0;
        imuDeltaAngle = 0.0;

        // Note: All other instance variables default to 0.0
    }

    private boolean eqWT(double val1, double val2, double e){
        return Math.abs(val1 - val2) <= e;
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

    /*public double[] calculateDifferentials(){
        double dxleft = -1.0 * measureLeftEncoderChange();
        double dxright = -1.0 *  measureRightEncoderChange();
        double dym = measureFrontEncoderChange();
        double[] measuredIMUAngle = imu.updateAngle();
        if((angleMode.equals("Encoder") && eqWT(dxleft, dxright, (Math.abs(dxleft) + Math.abs(dxright)) * .5))){
            dtheta = (dxright - dxleft) / c;
            dx = dym - forwardOffset * dtheta;
            dy = (dxleft + dxright) / 2;
        }
        else{
            //double s1 = dxleft;
            //double s2 = dxright;
            //double s3 = dym;

            //dx = (c* (s1/(s2-s1) + 1/2 ) + s3) * Math.cos((s2-s1)/c);
            //dy = (c* (s1/(s2-s1) + 1/2 ) + s3) * Math.sin((s2-s1)/c);
            if(angleMode.equals("Encoder")){
                dx = (((c * dxleft) / (dxright - dxleft)) + 0.5 * c + dym ) * (1 - Math.cos((dxright - dxleft) / c));
                dy = (((c * dxleft) / (dxright - dxleft)) + 0.5 * c + dym ) * (Math.sin((dxright - dxleft) / c));
                //dx = (c * (dxleft / (dxright - dxleft) + 0.5) + dym) * Math.cos((dxright - dxleft) / c);
                //dy = (c * (dxleft / (dxright - dxleft) + 0.5) + dym) * Math.sin((dxright - dxleft) / c);

                dtheta = (dxright - dxleft) / c;
            }
            else{


                dx = (((dxleft) / (measuredIMUAngle[1])) + 0.5 * c + dym ) * (1 - Math.cos(measuredIMUAngle[1]));
                dy = (((dxleft) / (measuredIMUAngle[1])) + 0.5 * c + dym ) * (Math.sin(measuredIMUAngle[1]));
                //dx = (c * (dxleft / (dxright - dxleft) + 0.5) + dym) * Math.cos((dxright - dxleft) / c);
                //dy = (c * (dxleft / (dxright - dxleft) + 0.5) + dym) * Math.sin((dxright - dxleft) / c);

            }

            imuAngleReading = measuredIMUAngle[0];
        }
        //}

        return new double[]{dx, dy, dtheta};
    }/*

     */

    public void calculateDifferentials(){
        double dL = -1.0 * measureLeftEncoderChange();
        double dR = -1.0 *  measureRightEncoderChange();
        double dF = measureFrontEncoderChange();
        dtheta = (dR - dL) / c;
        double[] gyroReading = imu.updateAngle();
        imuGlobalAngle = gyroReading[0];
        imuDeltaAngle = gyroReading[1];
        if(angleMode.equals("Encoder")){
            if (eqWT(dtheta, 0, .015)){
                dx = dF;
                dy = (dL + dR) / 2;
            }
            else{
                double rt = (c / 2) * (dL + dR) / (dR - dL);
                double rs = (dF / dtheta) + forwardOffset;
                dx = rt * (Math.cos(dtheta) - 1) + rs * Math.sin(dtheta);
                dy = rt * Math.sin(dtheta) + rs * (1 - Math.cos(dtheta));
            }
        }
        else{
            if (eqWT(imuDeltaAngle, 0, .015)){
                dx = dF;
                dy = (dL + dR) / 2;
            }
            else{
                double rt = (dL + dR) / (2 * imuDeltaAngle);
                double rs = (dF / imuDeltaAngle) + forwardOffset;
                dx = rt * (Math.cos(imuDeltaAngle) - 1) + rs * Math.sin(imuDeltaAngle);
                dy = rt * Math.sin(imuDeltaAngle) + rs * (1 - Math.cos(imuDeltaAngle));
            }
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
        if(angleMode.equals("Encoder")){theta += 0.5 * dtheta;}
        else{theta += 0.5 * imuDeltaAngle;}

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
            theta += 0.5 * dtheta;
        }
        else{
            theta = imuGlobalAngle;
        }
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