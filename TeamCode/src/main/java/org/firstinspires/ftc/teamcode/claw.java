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
        extender.setDirection(DcMotor.Direction.FORWARD);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void close(){
        leftTalon.setPosition(0);
        rightTalon.setPosition(0.4);
    }

    public void open(){
        leftTalon.setPosition(0.5);
        rightTalon.setPosition(0);
    }

    public void liftTo(int targetPos, double holdingForce){

        extender.setTargetPosition(targetPos);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(.5);

      /* int currentPos = extender.getCurrentPosition();
      if(currentPos < targetPos){
            while(currentPos < targetPos){
                extender.setPower(-1);
                currentPos = extender.getCurrentPosition();
            }
        }
        else if(currentPos > targetPos){
            while(currentPos > targetPos){
                extender.setPower(1);
                currentPos = extender.getCurrentPosition();
            }
        }
        extender.setPower(-1 * holdingForce);*/
    }

    public int getLiftPos(){ return extender.getCurrentPosition();}
}
