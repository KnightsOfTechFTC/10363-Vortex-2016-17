package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Lego5 on 10/23/2016.
 */
@Autonomous(name="10363 Competition Autonomous on Blue Alliance (right beacon)")
public class Team10363AutoBlueBeaconRight extends OpMode{

    Team_10363_Auto_Decs RobII= new Team_10363_Auto_Decs();
    @Override
    public void init() {
        RobII.init(hardwareMap);
        RobII.reset_drive_encoders();
        left_encoder = RobII.a_left_encoder_pos();
        right_encoder = RobII.a_right_encoder_pos();
    }

    @Override
    public void loop() {
        switch (v_state) {
            case 0:
                RobII.setDrivePower((float) (.5 - RobII.adjspeed(1, RobII.a_gyro_heading())), (float) (.5 + RobII.adjspeed(1, RobII.a_gyro_heading())));
                if (RobII.have_drive_encoders_reached(4657-left_encoder, 4657-right_encoder, true)){
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 1:
                RobII.setDrivePower(1, 0);
                if (RobII.a_gyro_heading() >= 45 && RobII.a_gyro_heading() < 180) {
                    RobII.setDrivePower(0, 0);
                    v_state++;
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                }
                break;
            case 2:
                RobII.setDrivePower((float) (.5 - RobII.adjspeed(1, RobII.a_gyro_heading()-45)), (float) (.5 + RobII.adjspeed(1, RobII.a_gyro_heading()-45)));
                if (RobII.a_ground_blue()>=2){
                    RobII.setDrivePower(0,0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                if (RobII.have_drive_encoders_reached(10363-left_encoder,10363-right_encoder,true)){
                    RobII.setDrivePower(0,0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state=v_state+2;
                }
                break;
            case 3:
                RobII.setDrivePower((float) (.5 - RobII.adjspeed(1, RobII.a_gyro_heading()-45)), (float) (.5 + RobII.adjspeed(1, RobII.a_gyro_heading()-45)));
                if (RobII.have_drive_encoders_reached(7763-left_encoder,7763-right_encoder,true)) {
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 4:
                RobII.setDrivePower(1,-1);
                if (RobII.a_gyro_heading()>=135 && RobII.a_gyro_heading()<180){
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 5:
                RobII.setDrivePower((float) (.5 - RobII.adjspeed(1, RobII.a_gyro_heading()-135)), (float) (.5 + RobII.adjspeed(1, RobII.a_gyro_heading()-135)));
                if (RobII.a_ground_alpha()>=8){
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 6:
                RobII.setDrivePower(-1,0);
                if (RobII.a_gyro_heading()>=90){
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;

                }
                break;
            case 7:
                //Robert's Jukes. Also called LineFollower because it follows the white line based on gyro and color sensors.
                if (RobII.a_ground_alpha()>8){
                    RobII.setDrivePower(0.5f,0.5f);}
                else if (RobII.a_gyro_heading()>90){RobII.setDrivePower(.7f,0);}
                else if (RobII.a_gyro_heading()<90){RobII.setDrivePower(0,.7f);}
                else {RobII.setDrivePower(.5f,.5f);}
                if (leftEnconderProblems == RobII.a_left_encoder_pos() && rightEnconderProblems == RobII.a_right_encoder_pos()){
                    count = count + 1;
                }else {
                    count = 0;
                    leftEnconderProblems = RobII.a_left_encoder_pos();
                    rightEnconderProblems = RobII.a_right_encoder_pos();
                }
                // Is counter greater than 126 iters (3*42)? If so, skip to next state.
                if (count > 126){
                    RobII.setDrivePower(0.0f, 0.0f);
                    v_state++;
                }
                else if (RobII.a_left_encoder_pos()-left_encoder+RobII.a_right_encoder_pos()-right_encoder>=7763){
                    RobII.setDrivePower(0.0f, 0.0f);
                    v_state++;
                }
            default:
                telemetry.update();
                telemetry.addData("200: ","Done!");
                break;
        }
    }
    int v_state=0;
    int left_encoder=0;
    int right_encoder=0;
    int count=0;
    int leftEnconderProblems=0;
    int rightEnconderProblems=0;



}
