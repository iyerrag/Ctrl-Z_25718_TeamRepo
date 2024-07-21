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
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public void liftTo(int targetPos) throws InterruptedException {
        extender.setTargetPosition(-1 * targetPos);
        extender.setPower(1);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(extender.isBusy()){}
        extender.setPower(0);
    }
}
