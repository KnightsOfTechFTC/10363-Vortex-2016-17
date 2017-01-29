package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Team_10363_TeleOp_Decs;

@TeleOp(name = "Team 10363 Single Stick")
public class Team_10363_TeleOp_single_stick_one_controller extends OpMode {
    /* RIP Robert the Robot 2015-2016. May his Res-Q skills be remembered by his 2 children,
        Robert II (a temp name) and JVBot (also a temp name) and his lifelong friend 9924Bot. */
    Team_10363_TeleOp_Decs RobertII= new Team_10363_TeleOp_Decs();
    @Override
    public void init() {

        RobertII.init(hardwareMap);
        telemetry.addData("  ",1);
        telemetry.update();
    }

    @Override
    public void loop() {
        //update telemetry
        telemetry.update();
        //set motor powers using single-stick driving methods
        leftPower=RobertII.single_stick_drive_left(gamepad1.left_stick_x,gamepad1.left_stick_y);
        rightPower=RobertII.single_stick_drive_right(gamepad1.left_stick_x,gamepad1.left_stick_y);
        if ((gamepad1.b&&!b_press)||(gamepad2.b&&!b_press)){
            b_press=true;
            slow=slow+.6;
            if (slow>1){slow=.4;}
        }
        if((gamepad1.b&&!b_press)&&(gamepad2.b&&!b_press)){
            b_press=false;
        }
        if (!gamepad1.b&&!gamepad2.b){
            b_press=false;
        }
        RobertII.setDrivePowerWithCorrection((float) (rightPower*slow), (float) (leftPower*slow));
        //add motor powers to telemetry
        telemetry.addData("0: Slow Mode Modifier: ", slow);
        telemetry.addData("1: Right Drive Motor Power: ", leftPower*slow);
        telemetry.addData("2: Left Drive Motor Power: ", rightPower*slow);

        if (gamepad1.dpad_up) {cap_updown = 1;}
        else if (gamepad1.dpad_down) {cap_updown = -1;}
        else {cap_updown=0;}

        RobertII.m_cap_power(cap_updown); //This is the cap lift modifier
//        RobertII.m_cap_power(gamepad1.right_stick_y);

        if ((gamepad1.b&&!b_press)||(gamepad2.b&&!b_press)){
            x_press=true;
            mode++;
            if(mode==2){mode=-1;}
        }
        if((gamepad1.x&&!x_press)&&(gamepad2.x&&!x_press)){
            x_press=false;
        }
        if (!gamepad1.x){
            x_press=false;
        }

        RobertII.m_intake_power(mode); //This is the intake modifier
        telemetry.addData("3: intake speed: ",mode);
        if ((gamepad1.y&&!y_press)||(gamepad2.y&&!y_press)){
            y_press=true;
            liftmode++;
            if(liftmode==2){liftmode=-1;}
        }
        if (gamepad1.x&&!x_press){
            x_press=true;
            mode++;
            if(mode==2){mode=-1;}
        }
        if (!gamepad1.x){
            x_press=false;
        }
        if (!gamepad1.y){
            y_press=false;
        }
        RobertII.m_lift_power(liftmode); //This is the ball life power modifier
        telemetry.addData("5: lift speed: ",liftmode);
        if ((gamepad1.a&&!a_press)||(gamepad2.a&&!a_press)){
            a_press=true;
            beacons=!beacons;
        }
        if ((gamepad1.a&&!a_press)&&(gamepad2.a&&!a_press)){
            a_press=false;
        }
//      if ((gamepad1.dpad_right&&!dpad_right)||(gamepad2.dpad_right&&!dpad_right)){
        if ((gamepad1.dpad_right)||(gamepad2.dpad_right)){
            beacon1=true;
        }
//        if ((gamepad1.dpad_right&&!dpad_right)&&(gamepad2.dpad_right&&!dpad_right)){
        if ((!gamepad1.dpad_right)&&(!gamepad2.dpad_right)){
            beacon1=false;
            telemetry.addData("Tube Beacon Status: ", beacon1);

        }
        if (gamepad1.right_trigger>gamepad2.right_trigger){
            RobertII.m_ball_shooting_power(gamepad1.right_trigger);
            telemetry.addData("Ball shooting Power",gamepad1.right_trigger);

        }else {
            RobertII.m_ball_shooting_power(gamepad2.right_trigger);
        }
        if(gamepad1.left_bumper){
            beacons1 = true;
            RobertII.beacon_extend(beacons1);
        }else if(gamepad1.right_bumper){
            beacons1 = false;
            RobertII.beacon_retract(beacons1);
        }else {
            RobertII.beacon_stop();
        }
        if (gamepad1.a||gamepad2.a){
            RobertII.m_lift_pos(-.1);
        }else if (gamepad1.y||gamepad2.y){
            RobertII.m_lift_pos(.1);
        }
    }
    // Variable Declarations
    double leftPower;
    double rightPower;
    int liftmode=0;
    int shootingmode = 0;
    double slow=1;
    boolean b_press;
    boolean y_press;
    boolean beacons;
    boolean beacon1;
    boolean dpad_right;
    boolean a_press;
    boolean beacons1;
    int mode=0;
    boolean x_press;
    boolean ball_press;
    int cap_updown=0;

}
