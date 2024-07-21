package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class claw {
    private Servo leftTalon;
    private Servo rightTalon;

    private DcMotor extender;


    public claw(Servo left, Servo right, DcMotor lifter){
        leftTalon = left;
        rightTalon = right;
        extender = lifter;
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setDirection(DcMotor.Direction.REVERSE);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void close(){
        leftTalon.setPosition(0);
        rightTalon.setPosition(0.4);
    }

    public void open(){
        leftTalon.setPosition(0.5);
        rightTalon.setPosition(0);
    }

    public void liftTo(int targetPos){
        extender.setTargetPosition(targetPos / 10);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(.1);
        while(extender.isBusy()){}
        extender.setPower(0);
    }

    public int getLiftPos(){ return extender.getCurrentPosition();}
}
