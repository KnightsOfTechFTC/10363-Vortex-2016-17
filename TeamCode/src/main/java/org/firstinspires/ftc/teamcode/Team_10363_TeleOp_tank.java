package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Lego5 on 11/5/2016.
 */
@TeleOp(name = "10363 Competition TeleOp (with Tank Drive)")
public class Team_10363_TeleOp_tank extends OpMode {
    /* RIP Robert the Robot 2015-2016. May his Res-Q skills be remembered by his 2 children,
        Robert II (a temp name) and JVBot (also a temp name) and his lifelong friend 9924Bot.  */
    Team_10363_TeleOp_Decs RobertII= new Team_10363_TeleOp_Decs();
    @Override
    public void init() {

        RobertII.init(hardwareMap);
    }

    @Override
    public void loop() {
        //update telemetry
        telemetry.update();
        //set motor powers using single-stick driving methods
        leftPower=gamepad1.right_stick_y;
        rightPower=gamepad1.left_stick_y;
            if (gamepad1.b&&!b_press){
            b_press=true;
            slow=slow+.6;
            if (slow>1){slow=.4;}
        }
        if (!gamepad1.b){
            b_press=false;
        }
        RobertII.setDrivePowerWithCorrection((float) (rightPower*slow), (float) (leftPower*slow));
        //add motor powers to telemetry
        telemetry.addData("0: Slow Mode Modifier: ", slow);
        telemetry.addData("1: Right Drive Motor Power: ", leftPower*slow);
        telemetry.addData("2: Left Drive Motor Power: ", rightPower*slow);
        if (gamepad1.x&&!x_press){
            x_press=true;
            mode++;
            if(mode==2){mode=-1;}
        }
        if (!gamepad1.x){
            x_press=false;
        }
        RobertII.m_intake_power(mode);
        telemetry.addData("3: intake speed: ",mode);
        if (gamepad1.y&&!y_press){
            y_press=true;
            liftmode++;
            if(liftmode==2){liftmode=-1;}
        }
        if (!gamepad1.y){
            y_press=false;
        }
        RobertII.m_lift_power(liftmode);
        telemetry.addData("4: lift speed: ",liftmode);
        if (gamepad1.a&&!a_press){
            a_press=true;
            beacons=!beacons;
        }
        if (!gamepad1.a){
            a_press=false;
        }
        RobertII.press_or_reset_beacons(beacons);
    }
    double leftPower;
    double rightPower;
    int liftmode=1;
    double slow=1;
    boolean b_press;
    boolean y_press;
    boolean beacons;
    boolean a_press;
    int mode=0;
    boolean x_press;

}
