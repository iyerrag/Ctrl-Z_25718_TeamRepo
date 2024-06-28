package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class robotIMU {
    static private BHI260IMU imu;
    static private BHI260IMU.Parameters parameters;
    static private Orientation lastAngles;
    static private double globalAngle;

    public robotIMU(BHI260IMU IMU){
        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu = IMU;

        imu.initialize(parameters);

        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0.0;
    }


    public double[] updateAngle(){
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if(deltaAngle < -180){
            deltaAngle += 360;
        }
        else if (deltaAngle > 180){
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return new double[]{globalAngle, deltaAngle};

    }
}
