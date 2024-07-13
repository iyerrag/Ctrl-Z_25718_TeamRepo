package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class claw {
    private Servo leftTalon;
    private Servo rightTalon;

    private DcMotor extender;

    public claw(Servo left, Servo right, DcMotor lifter){
        leftTalon = left;
        rightTalon = right;
        extender = lifter;
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void open(){
        leftTalon.setPosition(0.25);
        rightTalon.setPosition(0.25);
    }

    public void close(){
        leftTalon.setPosition(0);
        rightTalon.setPosition(0);
    }

    public void liftTo(int targetPos){
        extender.setTargetPosition(targetPos);
    }
}
