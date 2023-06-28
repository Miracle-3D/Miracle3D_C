#pragma once
//#include <vector>
#include <string>
class Params
{
private:
    Params();

    std::string wpDefault;
    std::string wpType0;
    std::string wpType1;
    std::string wpType2;
    std::string wpType3;
    std::string wpType4;

    std::string kpDajiao;
    std::string kpBigkick;
    std::string kpKick0;
    std::string kpKickik0;
    std::string kpKick;
    std::string kpLeftblock;
    std::string kpRightblock;
    std::string kpMiddleBlock;
    std::string kpLongpass;
    std::string kpSlowpass;
    std::string kpStand;

public:
    static Params &getInstance()
    {
        static Params instance = Params();
        return instance;
    }
    const std::string &getWalkParams(std::string name)
    {
        //cout<<name<<endl;
        if (name == "paramfiles/defaultParams.txt")
            return wpDefault;
        else if (name == "paramfiles/defaultParams_t0.txt")
            return wpType0;
        else if (name == "paramfiles/defaultParams_t1.txt")
            return wpType1;
        else if (name == "paramfiles/defaultParams_t2.txt")
            return wpType2;
        else if (name == "paramfiles/defaultParams_t3.txt")
            return wpType3;
        else if (name == "paramfiles/defaultParams_t4.txt")
            return wpType4;
        else
            throw "walk params name error";
    }
    const std::string &getKickParams(std::string name)
    {
        if (name == "./skills/dajiao.skl")
            return kpDajiao;
        else if (name == "./skills/big_kick.skl")
            return kpBigkick;
        else if (name == "./skills/kick_0.skl")
            return kpKick0;
        else if (name == "./skills/kick_ik_0.skl")
            return kpKickik0;
        else if (name == "./skills/kick.skl")
            return kpKick;
        else if (name == "./skills/left_block.skl")
            return kpLeftblock;
        else if (name == "./skills/right_block.skl")
            return kpRightblock;
        else if (name == "./skills/middle_block.skl")
            return kpMiddleBlock;
        else if (name == "./skills/longpass.skl")
            return kpLongpass;
        else if (name == "./skills/slowpass.skl")
            return kpSlowpass;
        else if (name == "./skills/stand.skl")
            return kpStand;
        else
            throw "kick params name error";
    }
};
inline Params::Params()
{
    wpDefault 
#pragma region    
= R"(### Default parameters loaded for all agents ###

##########################
### UT WALK PARAMETERS ###
##########################

##<--Walk parameters for going to a target-->##
// Maximum size of steps in radians and millimeters 
utwalk_max_step_size_angle	1.22
utwalk_max_step_size_x	50
utwalk_max_step_size_y	40
// How much center of mass is shifted from side to side during walk in millimeters
utwalk_shift_amount	20
// Height of the torso from ground in millimeters
utwalk_walk_height	175
// Maximum height of foot from ground in millimeters during step
utwalk_step_height	20
// Fraction of a phase the swing foot remains still before moving in the X-Y plane
utwalk_fraction_still	0.2
// Fraction of a phase that the swing foot spends on the ground before lifting
utwalk_fraction_on_ground	0.2
// Fraction  of a phase that the swing foot spends moving in the X-Y plane
utwalk_fraction_moving	0.6
// Fraction of a phase that the swing foot spends in the air
utwalk_fraction_in_air	0.6
// Duration of single step in seconds
utwalk_phase_length	0.38
// Expected difference between commanded COM and sensed COM
utwalk_default_com_pos_x	0.0
// Factors of how fast the step sizes change per time cycle (proportional controller)
utwalk_pid_step_size_x	0.03
utwalk_pid_step_size_y	0.03
utwalk_pid_step_size_rot	0.03
// Maximum COM error in millimeters before the steps are slowed
utwalk_max_normal_com_error	7.5
// Maximum COM error in millimeters before all velocity reach 0
utwalk_max_acceptable_com_error	12.5
// Constant offset between the torso and feet in millimeters
utwalk_fwd_offset	2.5
// Factor of the step size applied to the forwards position of the torso
utwalk_fwd_offset_factor	0.5
// Angle of foot when it hits the ground in radians
utwalk_swing_ankle_offset	-0.087266463
// Proportional controller values for the torso angles coming from IMU measurements
utwalk_pid_tilt	0.15
utwalk_pid_roll	0.2
// Proportional controller values for controlling the COM 
utwalk_pid_com_x	1.0
utwalk_pid_com_y	1.0
utwalk_pid_com_z	0.0
// Proportional controller values for moving arms to control COM
utwalk_pid_arm_x	0.0
utwalk_pid_arm_y	0.0

#--Default parameters for walk balance control--#
// Joint angle correction factors for body tilt and roll
utwalk_balance_hip_pitch	0
utwalk_balance_knee_pitch	0
utwalk_balance_hip_roll	0
utwalk_balance_ankle_roll	0

#--Default parameters for toe models--#
// Parameters for sinusoidal movement of toe and ankle pitch
utwalk_toe_const_offset	  0
utwalk_toe_amplitude	  0
utwalk_toe_phase_offset	  0
utwalk_ankle_const_offset	0
utwalk_ankle_amplitude	  0
utwalk_ankle_phase_offset	0

##<--Walk parameters for positioning/dribbling-->##
pos_utwalk_max_step_size_angle	1.22
pos_utwalk_max_step_size_x	50
pos_utwalk_max_step_size_y	40
pos_utwalk_shift_amount	20
pos_utwalk_walk_height	175
pos_utwalk_step_height	20
pos_utwalk_fraction_still	0.2
pos_utwalk_fraction_on_ground	0.2
pos_utwalk_fraction_moving	0.6
pos_utwalk_fraction_in_air	0.6
pos_utwalk_phase_length	0.38
pos_utwalk_default_com_pos_x	0.0
pos_utwalk_pid_step_size_x	0.03
pos_utwalk_pid_step_size_y	0.03
pos_utwalk_pid_step_size_rot	0.03
pos_utwalk_max_normal_com_error	7.5
pos_utwalk_max_acceptable_com_error	12.5
pos_utwalk_fwd_offset	2.5
pos_utwalk_fwd_offset_factor	0.5
pos_utwalk_swing_ankle_offset	-0.087266463
pos_utwalk_pid_tilt	0.15
pos_utwalk_pid_roll	0.2
pos_utwalk_pid_com_x	1.0
pos_utwalk_pid_com_y	1.0
pos_utwalk_pid_com_z	0.0
pos_utwalk_pid_arm_x	0.0
pos_utwalk_pid_arm_y	0.0
pos_utwalk_balance_hip_pitch	0
pos_utwalk_balance_knee_pitch	0
pos_utwalk_balance_hip_roll	0
pos_utwalk_balance_ankle_roll	0
pos_utwalk_toe_const_offset	  0
pos_utwalk_toe_amplitude	  0
pos_utwalk_toe_phase_offset	  0
pos_utwalk_ankle_const_offset	0
pos_utwalk_ankle_amplitude	  0
pos_utwalk_ankle_phase_offset	0

##<--Walk parameters for approaching the ball to kick-->##
app_utwalk_max_step_size_angle	1.22
app_utwalk_max_step_size_x	50
app_utwalk_max_step_size_y	40
app_utwalk_shift_amount	20
app_utwalk_walk_height	175
app_utwalk_step_height	20
app_utwalk_fraction_still	0.2
app_utwalk_fraction_on_ground	0.2
app_utwalk_fraction_moving	0.6
app_utwalk_fraction_in_air	0.6
app_utwalk_phase_length	0.38
app_utwalk_default_com_pos_x	0.0
app_utwalk_pid_step_size_x	0.03
app_utwalk_pid_step_size_y	0.03
app_utwalk_pid_step_size_rot	0.03
app_utwalk_max_normal_com_error	7.5
app_utwalk_max_acceptable_com_error	12.5
app_utwalk_fwd_offset	2.5
app_utwalk_fwd_offset_factor	0.5
app_utwalk_swing_ankle_offset	-0.087266463
app_utwalk_pid_tilt	0.15
app_utwalk_pid_roll	0.2
app_utwalk_pid_com_x	1.0
app_utwalk_pid_com_y	1.0
app_utwalk_pid_com_z	0.0
app_utwalk_pid_arm_x	0.0
app_utwalk_pid_arm_y	0.0
app_utwalk_balance_hip_pitch	0
app_utwalk_balance_knee_pitch	0
app_utwalk_balance_hip_roll	0
app_utwalk_balance_ankle_roll	0
app_utwalk_toe_const_offset	  0
app_utwalk_toe_amplitude	  0
app_utwalk_toe_phase_offset	  0
app_utwalk_ankle_const_offset	0
app_utwalk_ankle_amplitude	  0
app_utwalk_ankle_phase_offset	0


########################
### GETUP PARAMETERS ###
########################

# default getup off front params
getup_parms_stateDownInitialWait	0.5
getup_parms_stateDown3A1	30.0
getup_parms_stateDown3L3	110.0
getup_parms_stateDown3MinTime	0.04
getup_parms_stateDown5L1	-45.0
getup_parms_stateDown5MinTime	0.9
getup_parms_stateDown7L1	-25.0
getup_parms_stateDown7L3	60.0
getup_parms_stateDown7MinTime	0.3
getup_parms_stateDown10MinTime	0.8

# default getup off back params
getup_parms_stateUpInitialWait	0.5
getup_parms_stateUp3A1	-120.0
getup_parms_stateUp3A2	35.0
getup_parms_stateUp3A4	60.0
getup_parms_stateUp3L3	15.0
getup_parms_stateUp3MinTime	0.2
getup_parms_stateUp5L3	110.0
getup_parms_stateUp5MinTime	0.04
getup_parms_stateUp7L1	-35.0
getup_parms_stateUp7MinTime	0.2
getup_parms_stateUp9A1	30.0
getup_parms_stateUp9L1	-80.0
getup_parms_stateUp9L4	-60.0
getup_parms_stateUp9L5	-70.0
getup_parms_stateUp9L6	-40.0
getup_parms_stateUp9MinTime	0.2
getup_parms_stateUp11A1	30.0
getup_parms_stateUp11L1	-60.0 
getup_parms_stateUp11L5	-80.0
getup_parms_stateUp11MinTime	0.4
getup_parms_stateUp13A1	-90.0
getup_parms_stateUp13L1	-30.0
getup_parms_stateUp13L3	30.0
getup_parms_stateUp13L4	-10.0 
getup_parms_stateUp13L5	-10.0
getup_parms_stateUp13MinTime	0.04 
getup_parms_stateUp15MinTime	0.6


#######################
### KICK PARAMETERS ###
#######################

kick_p1	0
kick_p2	0
kick_p3	0
kick_p4	0
kick_p5	0
kick_p6	0
kick_p7	0
kick_p8	0
kick_p9	0
kick_p10	0
kick_p11	0
kick_p12	0
kick_p13	0
kick_p14	0
kick_p15	0
kick_p16	0
kick_p17	0
kick_p18	0
kick_p19	0
kick_p20	0
kick_p21	0
kick_p22	0
kick_p23	0
kick_p24	0
kick_p25	0
kick_p26	0
kick_p27	0
kick_p28	0
kick_p29	0
kick_p30	0
kick_p31	0
kick_p32	0
kick_p33	0
kick_p34	0
kick_p35	0
kick_p36	0
kick_p37	0
kick_p38	0
kick_p39	0
kick_p40	0
kick_p41	0
kick_p42	0
kick_p43	0
kick_p44	0
kick_p45	0
kick_p46	0
kick_p47	0
kick_p48	0
kick_p49	0
kick_p50	0
kick_p51	0
kick_p52	0
kick_p53	0
kick_p54	0
kick_p55	0
kick_p56	0
kick_p57	0
kick_p58	0
kick_p59	0
kick_p60	0
kick_p61	0
kick_p62	0
kick_p63	0
kick_p64	0
kick_p65	0
kick_p66	0
kick_p67	0
kick_p68	0
kick_p69	0
kick_p70	0
kick_p71	0
kick_p72	0
kick_p73	0
kick_p74	0
kick_p75	0
kick_p76	0
kick_p77	0
kick_p78	0
kick_p79	0
kick_p80	0
kick_p81	0
kick_p82	0
kick_p83	0
kick_p84	0
kick_p85	0
kick_p86	0
kick_p87	0
kick_p88	0
kick_p89	0
kick_p90	0
kick_p91	0
kick_p92	0
kick_p93	0
kick_p94	0
kick_p95	0
kick_p96	0
kick_p97	0
kick_p98	0
kick_p99	0
kick_p100	0
kick_p101	0
kick_p102	0
kick_p103	0
kick_p104	0
kick_p105	0
kick_p106	0
kick_p107	0
kick_p108	0
kick_p109	0
kick_p110	0
kick_p111	0
kick_p112	0
kick_p113	0
kick_p114	0
kick_p115	0
kick_p116	0
kick_p117	0
kick_xoffset	-0.18758631052473101
kick_yoffset	-0.08133614974057074
kick_scale1	5.295086273579072
kick_scale2	1.9096575829831766
kick_scale3	0.7896144070184505

kick_max_displacement_right	0.01
kick_max_displacement_left	0.01
kick_max_displacement_top	0.01
kick_max_displacement_bottom	0.01
kick_cw_angle_thresh	2
kick_ccw_angle_thresh	2
kick_angle	0


##########################
### IK KICK PARAMETERS ###
##########################

kick_ik_0_xoffset	-0.18184725746865413
kick_ik_0_yoffset	-0.007990019340567048
kick_ik_0_x0		0.09855534262963274
kick_ik_0_y0		0.04897226608420107
kick_ik_0_z0		0.06004895070570849
kick_ik_0_x1		-0.13267256199213984
kick_ik_0_y1		0.15055665409986765
kick_ik_0_z1		0.3048635084962904
kick_ik_0_x2		-0.075918848350498
kick_ik_0_y2		0.010843367764323163
kick_ik_0_z2		-0.03228058151402973
kick_ik_0_x3		0.3514121512894722
kick_ik_0_y3		-0.0915098467211551
kick_ik_0_z3		0.2932735025335922
kick_ik_0_a0		-2.0713675817098482
kick_ik_0_b0		4.168030311789961
kick_ik_0_c0		-0.17712625804502546
kick_ik_0_a1		-2.3258316746549554
kick_ik_0_b1		9.39335694003392
kick_ik_0_c1		-5.4878969788579175
kick_ik_0_a2		2.254184572289742
kick_ik_0_b2		0.014404161833793745
kick_ik_0_c2		-16.34929405684522
kick_ik_0_a3		-0.1703513663364682
kick_ik_0_b3		77.12670393386878
kick_ik_0_c3		-21.212384580007893
kick_ik_0_wait		0.06679452466769868
kick_ik_0_scale		2.434596016520202
kick_ik_0_off3_0	6.8002354818317885
kick_ik_0_off4_0	23.957167469656504
kick_ik_0_off5_0	-7.433399813693172
kick_ik_0_off3_1	-16.624470935986754
kick_ik_0_off4_1	20.351676522363075
kick_ik_0_off5_1	-25.63678390762887
kick_ik_0_off3_2	-50.00201321637502
kick_ik_0_off4_2	-39.33897746613399
kick_ik_0_off5_2	54.047464010320134

kick_ik_0_max_displacement_right	0.025
kick_ik_0_max_displacement_left	0.025
kick_ik_0_max_displacement_top	0.025
kick_ik_0_max_displacement_bottom	0.025
kick_ik_0_cw_angle_thresh	2
kick_ik_0_ccw_angle_thresh	2
kick_ik_0_angle		0


// Parameters for approaching the ball
kick_gen_approach_turnDist	0.25
kick_gen_approach_buff	0.05
kick_gen_approach_estVelCorrection	0.05
kick_gen_approach_navBallDist	0.5
kick_gen_approach_navBallCollision	0.25
kick_gen_approach_navBallAngle	40
kick_gen_approach_maxDecelX	0.5
kick_gen_approach_maxDecelY	0.5


// Parameters for dribbling
drib_coll_thresh		0.34
drib_target	0.18

##############################################################

#######################
### DAJIAO PARAMETERS ###
#######################

dajiao_xoffset	-0.17458631052473101
dajiao_yoffset	-0.012


dajiao_max_displacement_right	0.01
dajiao_max_displacement_left	0.01
dajiao_max_displacement_top	0.01
dajiao_max_displacement_bottom	0.01
dajiao_cw_angle_thresh	2
dajiao_ccw_angle_thresh	2
dajiao_angle	0




//longpass parameters

longpass_max_displacement_right	0.01
longpass_max_displacement_left	0.01
longpass_max_displacement_top	0.01
longpass_max_displacement_bottom	0.01
longpass_cw_angle_thresh	2
longpass_ccw_angle_thresh	2
longpass_angle	0

//longpass_xoffset	-0.16758631052473101
//longpass_yoffset	-0.01133614974057074


longpass_xoffset	-0.17458631052473101
longpass_yoffset	-0.012
//longpass_yoffset	-0.013

longpass_time0	0.5
longpass_time1	0.2
longpass_time2	0.2
longpass_time3	0.2
longpass_time4	0.4



//slowpass4 parameters
slowpass4_max_displacement_right	0.01
slowpass4_max_displacement_left	0.01
slowpass4_max_displacement_top	0.01
slowpass4_max_displacement_bottom	0.01
slowpass4_cw_angle_thresh	2
slowpass4_ccw_angle_thresh	2
slowpass4_angle	0


slowpass4_xoffset	-0.17458631052473101
slowpass4_yoffset	-0.012

# dajiao 默认参数
dajiao_la0	0
dajiao_ra0	0

dajiao_kick_w0	0.0
dajiao_kick_w1	0.0
dajiao_kick_w2	0.0
dajiao_kick_w3	0.0
dajiao_kick_w4	0.0
dajiao_kick_w5	0.0
dajiao_kick_w6	0.0
dajiao_kick_w7	0.0
dajiao_kick_w8	0.0
dajiao_kick_w9	0.0
dajiao_kick_w10	0.0
dajiao_kick_w11	0.0

dajiao_kick_j00	0.0
dajiao_kick_j01	0.0
dajiao_kick_j02	0.0
dajiao_kick_j03	0.0
dajiao_kick_j04	0.0
dajiao_kick_j05	0.0
dajiao_kick_j06	0.0
dajiao_kick_j07	0.0
dajiao_kick_j08	0.0
dajiao_kick_j09	0.0

dajiao_kick_j10	0.0
dajiao_kick_j11	0.0
dajiao_kick_j12	0.0
dajiao_kick_j13	0.0
dajiao_kick_j14	0.0
dajiao_kick_j15	0.0
dajiao_kick_j16	0.0
dajiao_kick_j17	0.0
dajiao_kick_j18	0.0

dajiao_kick_j30	0.0
dajiao_kick_j31	0.0
dajiao_kick_j32	0.0
dajiao_kick_j33	0.0
dajiao_kick_j34	0.0

dajiao_kick_j40	0.0
dajiao_kick_j41	0.0
dajiao_kick_j42	0.0
dajiao_kick_j43	0.0

dajiao_kick_j50	0.0
dajiao_kick_j51	0.0
dajiao_kick_j52	0.0
dajiao_kick_j53	0.0
dajiao_kick_j54	0.0
dajiao_kick_j55	0.0
dajiao_kick_j56	0.0
dajiao_kick_j57	0.0
dajiao_kick_j58	0.0

dajiao_kick_j60	0.0
dajiao_kick_j61	0.0
dajiao_kick_j62	0.0
dajiao_kick_j63	0.0
dajiao_kick_j64	0.0
dajiao_kick_j65	0.0
dajiao_kick_j66	0.0
dajiao_kick_j67	0.0
dajiao_kick_j68	0.0
)";
#pragma endregion
    wpType0
#pragma region
 = R"(
### Default parameters loaded for all type 0 agents ###

########################
### GETUP PARAMETERS ###
########################

# getup off front params - optimized for no self collisions
getup_parms_stateDownInitialWait	0.5
getup_parms_stateDown3A1	51.67747278227646
getup_parms_stateDown3L3	131.7535106037132
getup_parms_stateDown3MinTime	0.04
getup_parms_stateDown5L1	-45.61543864318751
getup_parms_stateDown5MinTime	0.9
getup_parms_stateDown7L1	7.146660948143131
getup_parms_stateDown7L3	64.86293984615665
getup_parms_stateDown7MinTime	0.3
getup_parms_stateDown10MinTime	0.8

# getup off back params - optimized for no self collisions
getup_parms_stateUpInitialWait	0.5
getup_parms_stateUp3A1	-137.82661679105144
getup_parms_stateUp3A2	47.75746255861538
getup_parms_stateUp3A4	62.08845222070459
getup_parms_stateUp3L3	20.129514885250764
getup_parms_stateUp3MinTime	0.2
getup_parms_stateUp5L3	118.20227245026733
getup_parms_stateUp5MinTime	0.04
getup_parms_stateUp7L1	-28.310230075504645
getup_parms_stateUp7MinTime	0.2
getup_parms_stateUp9A1	46.60215905763356
getup_parms_stateUp9L1	-90.29680084623156
getup_parms_stateUp9L4	-45.66593008367499
getup_parms_stateUp9L5	-47.3966607282542
getup_parms_stateUp9L6	-63.76376603815571
getup_parms_stateUp9MinTime	0.2
getup_parms_stateUp11A1	3.592540038790991
getup_parms_stateUp11L1	-60.16186063609875
getup_parms_stateUp11L5	-66.94418816067675
getup_parms_stateUp11MinTime	0.4
getup_parms_stateUp13A1	-95.06021619327173
getup_parms_stateUp13L1	-27.726371071345866
getup_parms_stateUp13L3	33.044752335210546
getup_parms_stateUp13L4	-29.541237439782954
getup_parms_stateUp13L5	-26.379841635865372
getup_parms_stateUp13MinTime	0.04 
getup_parms_stateUp15MinTime	0.6

#################################
### OPTIMIZED WALK PARAMETERS ###
#################################

##<--Walk parameters for going to a target-->##
/*
These values are not UT Austin Villa's regular walking to target parameters,
instead the team's parameters for positioning and dribbling are used as a 
placeholder.  See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
utwalk_max_step_size_angle	0.8545494920155963
utwalk_max_step_size_x	97.3370999925245
utwalk_max_step_size_y	86.05383913184158
utwalk_shift_amount	-6.28025527312195
utwalk_walk_height	164.73062881817913
utwalk_step_height	99.78154903105181
utwalk_fraction_still	0.3335746450599291
utwalk_fraction_on_ground	-0.02207231908990039
utwalk_phase_length	0.06252433781071613
utwalk_default_com_pos_x	-0.03388678863233263
utwalk_pid_step_size_x	0.015223307475066804
utwalk_pid_step_size_y	0.049536000043868426
utwalk_pid_step_size_rot	0.10935016019620925
utwalk_max_normal_com_error	29.541917028506017
utwalk_max_acceptable_com_error	168.19797533625513
utwalk_fwd_offset	5.383050376364819
utwalk_fwd_offset_factor	1.186447305495243
utwalk_fraction_moving	0.701917533891856
utwalk_fraction_in_air	1.1229075969791498
utwalk_swing_ankle_offset	-0.12017916982142124
utwalk_pid_tilt	0.19220698479426948
utwalk_pid_roll	0.0683336940436052
utwalk_pid_com_x	1.3388420239842675
utwalk_pid_com_y	1.0002121242348725
utwalk_pid_com_z	0.04401545283135938
utwalk_pid_arm_x	-0.25099065181854435
utwalk_pid_arm_y	-0.24379385047916316

##<--Walk parameters for positioning/dribbling-->##
/*
See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
pos_utwalk_max_step_size_angle	0.8545494920155963
pos_utwalk_max_step_size_x	97.3370999925245
pos_utwalk_max_step_size_y	86.05383913184158
pos_utwalk_shift_amount	-6.28025527312195
pos_utwalk_walk_height	164.73062881817913
pos_utwalk_step_height	99.78154903105181
pos_utwalk_fraction_still	0.3335746450599291
pos_utwalk_fraction_on_ground	-0.02207231908990039
pos_utwalk_phase_length	0.06252433781071613
pos_utwalk_default_com_pos_x	-0.03388678863233263
pos_utwalk_pid_step_size_x	0.015223307475066804
pos_utwalk_pid_step_size_y	0.049536000043868426
pos_utwalk_pid_step_size_rot	0.10935016019620925
pos_utwalk_max_normal_com_error	29.541917028506017
pos_utwalk_max_acceptable_com_error	168.19797533625513
pos_utwalk_fwd_offset	5.383050376364819
pos_utwalk_fwd_offset_factor	1.186447305495243
pos_utwalk_fraction_moving	0.701917533891856
pos_utwalk_fraction_in_air	1.1229075969791498
pos_utwalk_swing_ankle_offset	-0.12017916982142124
pos_utwalk_pid_tilt	0.19220698479426948
pos_utwalk_pid_roll	0.0683336940436052
pos_utwalk_pid_com_x	1.3388420239842675
pos_utwalk_pid_com_y	1.0002121242348725
pos_utwalk_pid_com_z	0.04401545283135938
pos_utwalk_pid_arm_x	-0.25099065181854435
pos_utwalk_pid_arm_y	-0.24379385047916316

##<--Walk parameters for approaching the ball to kick-->##
/*
See the following paper for how these parameters were optimized: 
---
UT Austin Villa: RoboCup 2014 3D Simulation League Competition and Technical Challenge Champions.
Patrick MacAlpine, Mike Depinet, Jason Liang, and Peter Stone.
In RoboCup-2014: Robot Soccer World Cup XVIII, Lecture Notes in Artificial Intelligence, Springer Verlag, Berlin, 2015.
---
*/
app_utwalk_max_step_size_angle	1.914342301823827
app_utwalk_max_step_size_x	77.82161854283214
app_utwalk_max_step_size_y	120.0385739568059
app_utwalk_shift_amount	-24.87050644857127
app_utwalk_walk_height	130.04739372718288
app_utwalk_step_height	69.38612254085031
app_utwalk_fraction_still	0.3462118423649482
app_utwalk_fraction_on_ground	-0.10841176091636043
app_utwalk_phase_length	0.06514615189378256
app_utwalk_default_com_pos_x	4.235859304091463
app_utwalk_pid_step_size_x	0.08002602229080781
app_utwalk_pid_step_size_y	0.05881124297085935
app_utwalk_pid_step_size_rot	0.11852037059223865
app_utwalk_max_normal_com_error	2.0271579821217482
app_utwalk_max_acceptable_com_error	163.40869269758235
app_utwalk_fwd_offset	5.697692970887522
app_utwalk_fwd_offset_factor	1.0967555332068983
app_utwalk_fraction_moving	0.5962830272751801
app_utwalk_fraction_in_air	1.2489745680191913
app_utwalk_swing_ankle_offset	-0.11628068587478745
app_utwalk_pid_tilt	0.19416815389258132
app_utwalk_pid_roll	0.023839190106379415
app_utwalk_pid_com_x	1.5924070229564757
app_utwalk_pid_com_y	0.845651433093578
app_utwalk_pid_com_z	0.14853330612892843
app_utwalk_pid_arm_x	-0.3295152103093948
app_utwalk_pid_arm_y	-0.31023556762286403


// Parameters for approaching the ball
kick_gen_approach_turnDist	0.2729319251266597
kick_gen_approach_buff	0.010340912721421135
kick_gen_approach_estVelCorrection	-0.10445818587148002
kick_gen_approach_navBallDist	0.5
kick_gen_approach_navBallCollision	0.333
//kick_gen_approach_navBallAngle	37.67637845256782
kick_gen_approach_navBallAngle	20.0
kick_gen_approach_maxDecelX	0.75
kick_gen_approach_maxDecelY	0.8861122500586298


// Parameters for dribbling
drib_coll_thresh    0.33582030312352373
drib_target 0.17785682250297227


# t0 dajiao 不稳定
/*
dajiao_kick_w0	-0.5275089835835519
dajiao_kick_w1	0.22926781484421233
dajiao_kick_w2	0.02811868415838597
dajiao_kick_w3	-1.2835932837268336
dajiao_kick_w4	0.045965429522211707
dajiao_kick_w5	-2.0359120280043386
dajiao_kick_w6	0.09779622792868553
dajiao_kick_w7	-1.4163123535253934
dajiao_kick_w8	0.4469520396059935
dajiao_kick_w9	-1.815020769198701
dajiao_kick_w10	0.008260277419732622
dajiao_kick_w11	1.0219428581025927


dajiao_kick_j00	-1.7767255461224634
dajiao_kick_j01	-1.3714184991770368
dajiao_kick_j02	-4.216163394128136
dajiao_kick_j03	-3.286626053786059
dajiao_kick_j04	-6.89768359312551
dajiao_kick_j05	0.6242558399863256
dajiao_kick_j06	0.615867123540657
dajiao_kick_j07	5.294451635777307
dajiao_kick_j08	-0.14671566071824294
dajiao_kick_j09	5.6480436933172165
dajiao_kick_j10	-4.769143433677399
dajiao_kick_j11	7.161720686625263
dajiao_kick_j12	1.3502920416541024
dajiao_kick_j13	-3.2023995055886956
dajiao_kick_j14	3.9371440368240203
dajiao_kick_j15	3.138193279202598
dajiao_kick_j16	-1.7878857480470751
dajiao_kick_j17	9.847745085715943
dajiao_kick_j18	-1.1957268457920471
dajiao_kick_j30	3.204162610849501
dajiao_kick_j31	6.164558183991472
dajiao_kick_j32	0.910327253567502
dajiao_kick_j33	-3.7495409806894924
dajiao_kick_j34	1.8974131891666257
dajiao_kick_j40	1.177377684139411
dajiao_kick_j41	-0.2085352819967179
dajiao_kick_j42	-1.5024468496879941
dajiao_kick_j43	-0.5502003783030132
dajiao_kick_j50	-1.8454022979244464
dajiao_kick_j51	-3.9058894887311544
dajiao_kick_j52	-0.474915285156429
dajiao_kick_j53	-0.14979019830665985
dajiao_kick_j54	-4.633130422038536
dajiao_kick_j55	-4.772124087829191
dajiao_kick_j56	0.10265024032898679
dajiao_kick_j57	4.456452899145574
dajiao_kick_j58	-7.350604267597808
dajiao_kick_j60	-3.122814171273307
dajiao_kick_j61	1.6693205508145073
dajiao_kick_j62	-2.7767122363464796
dajiao_kick_j63	0.21751507205551435
dajiao_kick_j64	1.3413145045151935
dajiao_kick_j65	4.790741523105395
dajiao_kick_j66	0.7680799136418471
dajiao_kick_j67	4.650659575705593
dajiao_kick_j68	0.4742091462645839
*/

#t0 walk 6m 不稳定
/*
utwalk_default_com_pos_x	-0.30027024508514605
utwalk_fraction_in_air	1.1252269695896704
utwalk_fraction_moving	1.0216696075953102
utwalk_fraction_on_ground	-0.0649225095267473
utwalk_fraction_still	0.3714645313118197
utwalk_fwd_offset	5.574001013055146
utwalk_fwd_offset_factor	1.630533920663479
utwalk_max_acceptable_com_error	168.34817407247508
utwalk_max_normal_com_error	29.94629532452899
utwalk_max_step_size_angle	0.9167964988929761
utwalk_max_step_size_x	97.71853492042257
utwalk_max_step_size_y	85.87642672909945
utwalk_phase_length	0.0516883147332445
utwalk_step_height	99.46723047376314
utwalk_swing_ankle_offset	0.1668451366712801
utwalk_walk_height	164.6969859004778
*/



#t0 kickik 
kick_ik_0_xoffset	-0.14090799473448515
kick_ik_0_yoffset	-0.053709927307494985
kick_ik_0_x0	0.09221595270743028
kick_ik_0_y0	0.01567389876349559
kick_ik_0_z0	0.06333606682986123
kick_ik_0_x1	-0.08161518292570086
kick_ik_0_y1	0.17282125513642732
kick_ik_0_z1	0.32723824427569875
kick_ik_0_x2	-0.011568452830398437
kick_ik_0_y2	0.03549411499994546
kick_ik_0_z2	0.023144300551530772
kick_ik_0_x3	0.4048650387064445
kick_ik_0_y3	-0.11327268578343211
kick_ik_0_z3	0.28515477156702795
kick_ik_0_a0	-2.022236095789945
kick_ik_0_b0	4.21086290684667
kick_ik_0_c0	-0.12293189015060546
kick_ik_0_a1	-2.330578763228667
kick_ik_0_b1	9.410571105421422
kick_ik_0_c1	-5.401291066217618
kick_ik_0_a2	2.2928026456063666
kick_ik_0_b2	0.02611959140767659
kick_ik_0_c2	-16.358342067869252
kick_ik_0_a3	-0.1856277063846809
kick_ik_0_b3	77.22013767653175
kick_ik_0_c3	-21.239752305785103
kick_ik_0_wait	0.05783679869300602
kick_ik_0_scale	2.515297155023697
kick_ik_0_off3_0	6.768626895175123
kick_ik_0_off4_0	23.944014619700102
kick_ik_0_off5_0	-7.410032479983222
kick_ik_0_off3_1	-16.685463292944096
kick_ik_0_off4_1	20.37903747128188
kick_ik_0_off5_1	-25.684045078667857
kick_ik_0_off3_2	-50.0473676354327
kick_ik_0_off4_2	-39.40585360351973
kick_ik_0_off5_2	54.07512961060784




 )";
#pragma endregion
    wpType1
#pragma region
 = R"(
### Default parameters loaded for all type 1 agents ###

########################
### GETUP PARAMETERS ###
########################

# getup off front params - optimized for no self collisions
getup_parms_stateDownInitialWait	0.5
getup_parms_stateDown3A1	16.568148350107915
getup_parms_stateDown3L3	113.41133450590875
getup_parms_stateDown3MinTime	0.04
getup_parms_stateDown5L1	-39.17965858391402
getup_parms_stateDown5MinTime	0.9
getup_parms_stateDown7L1	-8.232686646848567
getup_parms_stateDown7L3	62.10471861129633
getup_parms_stateDown7MinTime	0.3
getup_parms_stateDown10MinTime	0.8

# getup off back params - optimized for no self collisions
getup_parms_stateUpInitialWait	0.5
getup_parms_stateUp3A1	-165.16130968225886
getup_parms_stateUp3A2	42.58511262180544
getup_parms_stateUp3A4	117.07281960213916
getup_parms_stateUp3L3	55.45054826070052
getup_parms_stateUp3MinTime	0.2
getup_parms_stateUp5L3	115.49895139205304
getup_parms_stateUp5MinTime	0.04
getup_parms_stateUp7L1	-68.1166661743677
getup_parms_stateUp7MinTime	0.2
getup_parms_stateUp9A1	46.81694893244553
getup_parms_stateUp9L1	-122.97859206412672
getup_parms_stateUp9L4	-37.42441472881181
getup_parms_stateUp9L5	-69.18081712780187
getup_parms_stateUp9L6	-86.80191982495745
getup_parms_stateUp9MinTime	0.2
getup_parms_stateUp11A1	42.78745493987175
getup_parms_stateUp11L1	-63.476728163679795
getup_parms_stateUp11L5	-92.55995382987949
getup_parms_stateUp11MinTime	0.4
getup_parms_stateUp13A1	-50.62380061268099
getup_parms_stateUp13L1	-27.678137746769536
getup_parms_stateUp13L3	18.485095263513564
getup_parms_stateUp13L4	14.273184019175599
getup_parms_stateUp13L5	-58.9003662416826
getup_parms_stateUp13MinTime	0.04 
getup_parms_stateUp15MinTime	0.6


#######################
### KICK PARAMETERS ###
#######################

kick_p1	-21.878452192908963
kick_p2	-107.2655684773586
kick_p3	-46.03084224893723
kick_p4	122.50096005151583
kick_p5	36.70802042526406
kick_p6	74.34964372370875
kick_p7	-23.35326324263892
kick_p8	-1.411851711850464
kick_p9	4.8436144712052585
kick_p11	-12.878536016473056
kick_p12	8.73474032972045
kick_p13	-2.0087222938685585
kick_p14	2.4649510821897755
kick_xoffset	-0.19721363030046984
kick_yoffset	-0.07855676227334851
kick_scale1	7.790202814460635
kick_scale2	5.980745105156915
kick_scale3	0.60539945683587


#################################
### OPTIMIZED WALK PARAMETERS ###
#################################

##<--Walk parameters for going to a target-->##
/*
These values are not UT Austin Villa's regular walking to target parameters,
instead the team's parameters for positioning and dribbling are used as a 
placeholder.  See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
utwalk_max_step_size_angle	0.8024586090513253
utwalk_max_step_size_x	108.7556110915902
utwalk_max_step_size_y	74.4935510327151
utwalk_shift_amount	-18.294447081916328
utwalk_walk_height	149.02400810686044
utwalk_step_height	77.51792538836638
utwalk_fraction_still	0.3434245549175683
utwalk_fraction_on_ground	-0.05778051267537769
utwalk_phase_length	0.06196439965990527
utwalk_default_com_pos_x	-49.92404794567735
utwalk_pid_step_size_x	0.02913534549352995
utwalk_pid_step_size_y	0.040069251483028374
utwalk_pid_step_size_rot	0.08311984252228707
utwalk_max_normal_com_error	14.173965772864628
utwalk_max_acceptable_com_error	99.70250710214097
utwalk_fwd_offset	1.620534424237782
utwalk_fwd_offset_factor	0.8878126276949263
utwalk_fraction_moving	0.7066620087173812
utwalk_fraction_in_air	1.2265865010025827
utwalk_swing_ankle_offset	-0.1276370985800854
utwalk_pid_tilt	0.23481727883696232
utwalk_pid_roll	-0.13254176315494343
utwalk_pid_com_x	1.2839211224476685
utwalk_pid_com_y	0.710246393252769
utwalk_pid_com_z	0.09199521532665715
utwalk_pid_arm_x	-0.28009850961801286
utwalk_pid_arm_y	-0.15921680247098174

##<--Walk parameters for positioning/dribbling-->##
/*
See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
pos_utwalk_max_step_size_angle	0.8024586090513253
pos_utwalk_max_step_size_x	108.7556110915902
pos_utwalk_max_step_size_y	74.4935510327151
pos_utwalk_shift_amount	-18.294447081916328
pos_utwalk_walk_height	149.02400810686044
pos_utwalk_step_height	77.51792538836638
pos_utwalk_fraction_still	0.3434245549175683
pos_utwalk_fraction_on_ground	-0.05778051267537769
pos_utwalk_phase_length	0.06196439965990527
pos_utwalk_default_com_pos_x	-49.92404794567735
pos_utwalk_pid_step_size_x	0.02913534549352995
pos_utwalk_pid_step_size_y	0.040069251483028374
pos_utwalk_pid_step_size_rot	0.08311984252228707
pos_utwalk_max_normal_com_error	14.173965772864628
pos_utwalk_max_acceptable_com_error	99.70250710214097
pos_utwalk_fwd_offset	1.620534424237782
pos_utwalk_fwd_offset_factor	0.8878126276949263
pos_utwalk_fraction_moving	0.7066620087173812
pos_utwalk_fraction_in_air	1.2265865010025827
pos_utwalk_swing_ankle_offset	-0.1276370985800854
pos_utwalk_pid_tilt	0.23481727883696232
pos_utwalk_pid_roll	-0.13254176315494343
pos_utwalk_pid_com_x	1.2839211224476685
pos_utwalk_pid_com_y	0.710246393252769
pos_utwalk_pid_com_z	0.09199521532665715
pos_utwalk_pid_arm_x	-0.28009850961801286
pos_utwalk_pid_arm_y	-0.15921680247098174

##<--Walk parameters for approaching the ball to kick-->##
/*
See the following paper for how these parameters were optimized: 
---
UT Austin Villa: RoboCup 2014 3D Simulation League Competition and Technical Challenge Champions.
Patrick MacAlpine, Mike Depinet, Jason Liang, and Peter Stone.
In RoboCup-2014: Robot Soccer World Cup XVIII, Lecture Notes in Artificial Intelligence, Springer Verlag, Berlin, 2015.
---
*/
app_utwalk_max_step_size_angle	1.9381533235593797
app_utwalk_max_step_size_x	71.63582250349182
app_utwalk_max_step_size_y	144.900163304773
app_utwalk_shift_amount	-21.907349427837936
app_utwalk_walk_height	124.01200316628147
app_utwalk_step_height	73.61066724376514
app_utwalk_fraction_still	0.29939200115833986
app_utwalk_fraction_on_ground	-0.18387526959605313
app_utwalk_phase_length	0.06807206697299917
app_utwalk_default_com_pos_x	-0.4084305647968729
app_utwalk_pid_step_size_x	0.13707529143391334
app_utwalk_pid_step_size_y	0.046927757158147856
app_utwalk_pid_step_size_rot	0.12279629485746335
app_utwalk_max_normal_com_error	46.11273420905612
app_utwalk_max_acceptable_com_error	77.74540513509403
app_utwalk_fwd_offset	0.13176552428735822
app_utwalk_fwd_offset_factor	0.9684198651409902
app_utwalk_fraction_moving	0.6237845064716534
app_utwalk_fraction_in_air	1.317851312205039
app_utwalk_swing_ankle_offset	-0.33789073267058567
app_utwalk_pid_tilt	0.3581152193771387
app_utwalk_pid_roll	-0.23108588849633932
app_utwalk_pid_com_x	1.2931738443483614
app_utwalk_pid_com_y	0.48451991950515516
app_utwalk_pid_com_z	0.19893585988034976
app_utwalk_pid_arm_x	-0.4310964222520728
app_utwalk_pid_arm_y	-0.5753525616637605


// Parameters for approaching the ball
kick_gen_approach_turnDist	0.22650837898147766
kick_gen_approach_buff	0.3335386081348905
kick_gen_approach_estVelCorrection	-0.05248222665309938
kick_gen_approach_navBallDist	0.5
kick_gen_approach_navBallCollision	0.333
//kick_gen_approach_navBallAngle	22.878438874871343
kick_gen_approach_navBallAngle	20.0
kick_gen_approach_maxDecelX	0.7930036316433737
kick_gen_approach_maxDecelY	0.3368459512499765


// Parameters for dribbling
drib_coll_thresh				0.33582030312352373
drib_target					0.17785682250297227



# kickik
kick_ik_0_xoffset	-0.15682081261276723
kick_ik_0_yoffset	-0.008861143437747595
kick_ik_0_x0	0.09315829496119322
kick_ik_0_y0	0.044026873199038814
kick_ik_0_z0	0.08993272478633167
kick_ik_0_x1	-0.1499430721157246
kick_ik_0_y1	0.16928521259364926
kick_ik_0_z1	0.33231548125211513
kick_ik_0_x2	-0.06403404807322355
kick_ik_0_y2	-0.0777957879459716
kick_ik_0_z2	0.021085904591344358
kick_ik_0_x3	0.3238727151523294
kick_ik_0_y3	-0.1341378705025853
kick_ik_0_z3	0.28351563854319756
kick_ik_0_a0	-2.047891529493969
kick_ik_0_b0	4.191242028966208
kick_ik_0_c0	-0.15233731131729297
kick_ik_0_a1	-2.302171832784579
kick_ik_0_b1	9.355770173068443
kick_ik_0_c1	-5.506915157310711
kick_ik_0_a2	2.284590332987274
kick_ik_0_b2	0.004021471067912419
kick_ik_0_c2	-16.326945041540863
kick_ik_0_a3	-0.20286923805404186
kick_ik_0_b3	77.16440705259224
kick_ik_0_c3	-21.22012167813849
kick_ik_0_wait	0.08434514310187124
kick_ik_0_scale	2.464629395444756
kick_ik_0_off3_0	6.79152574404598
kick_ik_0_off4_0	23.93392749142096
kick_ik_0_off5_0	-7.412367875488356
kick_ik_0_off3_1	-16.66227540972807
kick_ik_0_off4_1	20.268862244101395
kick_ik_0_off5_1	-25.634960604986844
kick_ik_0_off3_2	-50.02999949792415
kick_ik_0_off4_2	-39.377927496018195
kick_ik_0_off5_2	54.04596762966703

#dajiao
dajiao_kick_j00	0.925157942031053
dajiao_kick_j01	0.827841169687884
dajiao_kick_j02	1.3605728805382014
dajiao_kick_j03	0.6693418453996953
dajiao_kick_j04	-0.8681887770751685
dajiao_kick_j05	-1.0387230765059152
dajiao_kick_j06	3.3281494193279926
dajiao_kick_j07	-0.7722272091736486
dajiao_kick_j08	-0.0799943368992862
dajiao_kick_j09	-0.537105233049934
dajiao_kick_j10	2.221521554043446
dajiao_kick_j11	1.6645074168569127
dajiao_kick_j12	-0.6428305967408654
dajiao_kick_j13	0.19167040294849036
dajiao_kick_j14	0.051239184350639344
dajiao_kick_j15	0.13366095807053713
dajiao_kick_j16	1.1220120026096814
dajiao_kick_j17	0.5375727392528118
dajiao_kick_j18	2.956282417036217
dajiao_kick_j30	-0.9902578289551235
dajiao_kick_j31	-0.3205667947884927
dajiao_kick_j32	0.18634443850125948
dajiao_kick_j33	-1.3198399828818996
dajiao_kick_j34	-0.7562346797859281
dajiao_kick_j40	0.29566769022542794
dajiao_kick_j41	0.5091052242107647
dajiao_kick_j42	-0.41282692846585
dajiao_kick_j43	-0.5052061966758861
dajiao_kick_j50	0.6629701194773483
dajiao_kick_j51	0.8312135632998734
dajiao_kick_j52	0.12869816357080943
dajiao_kick_j53	-0.9719821587660262
dajiao_kick_j54	-2.0152348852126556
dajiao_kick_j55	-2.429160186626732
dajiao_kick_j56	-1.701388993461439
dajiao_kick_j57	-1.6141181130359534
dajiao_kick_j58	-2.1435578531455945
dajiao_kick_j60	-2.9734566505753097
dajiao_kick_j61	1.4423602222425838
dajiao_kick_j62	0.14647114650912602
dajiao_kick_j63	-1.9002037333882364
dajiao_kick_j64	1.0992602922177308
dajiao_kick_j65	-1.144106734676038
dajiao_kick_j66	-0.25387338487845545
dajiao_kick_j67	-1.44110216550243
dajiao_kick_j68	-1.2836370982612795


)";
#pragma endregion
    wpType2
#pragma region
 = R"(### Default parameters loaded for all type 2 agents ###

########################
### GETUP PARAMETERS ###
########################

# getup off front params - optimized for no self collisions
getup_parms_stateDownInitialWait	0.5
getup_parms_stateDown3A1	47.4610405996882
getup_parms_stateDown3L3	104.58701666386462
getup_parms_stateDown3MinTime	0.04
getup_parms_stateDown5L1	-46.16626996041264
getup_parms_stateDown5MinTime	0.9
getup_parms_stateDown7L1	-8.174758505626324
getup_parms_stateDown7L3	66.7418834956474
getup_parms_stateDown7MinTime	0.3
getup_parms_stateDown10MinTime	0.8

# getup off back params - optimized for no self collisions
getup_parms_stateUpInitialWait	0.5
getup_parms_stateUp3A1	-139.74945449732297
getup_parms_stateUp3A2	39.25063221342788
getup_parms_stateUp3A4	73.43239538646863
getup_parms_stateUp3L3	17.86850643399282
getup_parms_stateUp3MinTime	0.2
getup_parms_stateUp5L3	123.30024625855616
getup_parms_stateUp5MinTime	0.04
getup_parms_stateUp7L1	-38.67056678443029
getup_parms_stateUp7MinTime	0.2
getup_parms_stateUp9A1	56.9077493498944
getup_parms_stateUp9L1	-99.67874785395259
getup_parms_stateUp9L4	-33.532000140847295
getup_parms_stateUp9L5	-60.93062460878395
getup_parms_stateUp9L6	-70.7774560996261
getup_parms_stateUp9MinTime	0.2
getup_parms_stateUp11A1	45.0442618099886
getup_parms_stateUp11L1	-58.43623407729397
getup_parms_stateUp11L5	-93.22374982305332
getup_parms_stateUp11MinTime	0.4
getup_parms_stateUp13A1	-95.0269318603912
getup_parms_stateUp13L1	-13.040875418712943
getup_parms_stateUp13L3	48.91105714103771
getup_parms_stateUp13L4	-7.835010869903101
getup_parms_stateUp13L5	-59.141845254226816
getup_parms_stateUp13MinTime	0.04 
getup_parms_stateUp15MinTime	0.6


#######################
### KICK PARAMETERS ###
#######################

kick_p1	-16.155321891055568
kick_p2	-118.94574002268958
kick_p3	-49.0573368618613
kick_p4	114.49734317000089
kick_p5	27.093511567801276
kick_p6	56.65895221759296
kick_p7	5.333153982193821
kick_p8	-7.881565931921614
kick_p9	-3.103131585235596
kick_p11	-1.2098751103915228
kick_p12	5.000775182873085
kick_p13	-6.017267272068568
kick_p14	3.6943627753113
kick_xoffset	-0.16411604294659987
kick_yoffset	-0.08084359577763695
kick_scale1	6.483680425814427
kick_scale2	3.7629696858600292
kick_scale3	1.427020919469831


#################################
### OPTIMIZED WALK PARAMETERS ###
#################################

##<--Walk parameters for going to a target-->##
/*
These values are not UT Austin Villa's regular walking to target parameters,
instead the team's parameters for positioning and dribbling are used as a 
placeholder.  See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
utwalk_max_step_size_angle	1.4334029684506016
utwalk_max_step_size_x	188.3672476033237
utwalk_max_step_size_y	101.50014548282277
utwalk_shift_amount	-18.250369857080912
utwalk_walk_height	232.98538358428863
utwalk_step_height	60.42918023513916
utwalk_fraction_still	0.05321199357333965
utwalk_fraction_on_ground	-0.7888506445924227
utwalk_phase_length	0.0448837562360429
utwalk_default_com_pos_x	-47.509227978648184
utwalk_pid_step_size_x	0.019600539451164974
utwalk_pid_step_size_y	0.05729341617681754
utwalk_pid_step_size_rot	0.07977702013496352
utwalk_max_normal_com_error	-35.31432916993949
utwalk_max_acceptable_com_error	145.6468542047311
utwalk_fwd_offset	0.10614846471791811
utwalk_fwd_offset_factor	0.5881965176480436
utwalk_fraction_moving	0.6831197098033956
utwalk_fraction_in_air	1.6739521579648524
utwalk_swing_ankle_offset	-0.06027579444559555
utwalk_pid_tilt	-0.01781237530616678
utwalk_pid_roll	0.17175432973973306
utwalk_pid_com_x	1.3915113098858551
utwalk_pid_com_y	0.8485088262769881
utwalk_pid_com_z	-0.20204972969327478
utwalk_pid_arm_x	-0.4194139464825001
utwalk_pid_arm_y	-0.2538344873898322

##<--Walk parameters for positioning/dribbling-->##
/*
See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
pos_utwalk_max_step_size_angle	1.4334029684506016
pos_utwalk_max_step_size_x	188.3672476033237
pos_utwalk_max_step_size_y	101.50014548282277
pos_utwalk_shift_amount	-18.250369857080912
pos_utwalk_walk_height	232.98538358428863
pos_utwalk_step_height	60.42918023513916
pos_utwalk_fraction_still	0.05321199357333965
pos_utwalk_fraction_on_ground	-0.7888506445924227
pos_utwalk_phase_length	0.0448837562360429
pos_utwalk_default_com_pos_x	-47.509227978648184
pos_utwalk_pid_step_size_x	0.019600539451164974
pos_utwalk_pid_step_size_y	0.05729341617681754
pos_utwalk_pid_step_size_rot	0.07977702013496352
pos_utwalk_max_normal_com_error	-35.31432916993949
pos_utwalk_max_acceptable_com_error	145.6468542047311
pos_utwalk_fwd_offset	0.10614846471791811
pos_utwalk_fwd_offset_factor	0.5881965176480436
pos_utwalk_fraction_moving	0.6831197098033956
pos_utwalk_fraction_in_air	1.6739521579648524
pos_utwalk_swing_ankle_offset	-0.06027579444559555
pos_utwalk_pid_tilt	-0.01781237530616678
pos_utwalk_pid_roll	0.17175432973973306
pos_utwalk_pid_com_x	1.3915113098858551
pos_utwalk_pid_com_y	0.8485088262769881
pos_utwalk_pid_com_z	-0.20204972969327478
pos_utwalk_pid_arm_x	-0.4194139464825001
pos_utwalk_pid_arm_y	-0.2538344873898322

##<--Walk parameters for approaching the ball to kick-->##
/*
See the following paper for how these parameters were optimized: 
---
UT Austin Villa: RoboCup 2014 3D Simulation League Competition and Technical Challenge Champions.
Patrick MacAlpine, Mike Depinet, Jason Liang, and Peter Stone.
In RoboCup-2014: Robot Soccer World Cup XVIII, Lecture Notes in Artificial Intelligence, Springer Verlag, Berlin, 2015.
---
*/
app_utwalk_max_step_size_angle	2.494357540527233
app_utwalk_max_step_size_x	104.3757203589217
app_utwalk_max_step_size_y	159.38928916333188
app_utwalk_shift_amount	-17.01319088600292
app_utwalk_walk_height	219.58475968855998
app_utwalk_step_height	71.19566161941542
app_utwalk_fraction_still	0.24852431363522712
app_utwalk_fraction_on_ground	-0.7762872258330127
app_utwalk_phase_length	0.04134039859096437
app_utwalk_default_com_pos_x	-69.57120307416017
app_utwalk_pid_step_size_x	0.11622260186093741
app_utwalk_pid_step_size_y	0.05547735966747566
app_utwalk_pid_step_size_rot	0.1276910695197559
app_utwalk_max_normal_com_error	-6.678102010587821
app_utwalk_max_acceptable_com_error	202.3380012294417
app_utwalk_fwd_offset	-5.074390718305334
app_utwalk_fwd_offset_factor	0.7543429776922196
app_utwalk_fraction_moving	0.6625831141708978
app_utwalk_fraction_in_air	1.8417109278291766
app_utwalk_swing_ankle_offset	-0.13426995890217877
app_utwalk_pid_tilt	0.0428529701904648
app_utwalk_pid_roll	0.1811706152486411
app_utwalk_pid_com_x	1.3767754534308194
app_utwalk_pid_com_y	0.8505022730386051
app_utwalk_pid_com_z	-0.2164029576260493
app_utwalk_pid_arm_x	-0.28977951773581695
app_utwalk_pid_arm_y	-0.35056541007888586


// Parameters for approaching the ball
kick_gen_approach_turnDist	0.2746380969510346
kick_gen_approach_buff	0.33743214186481857
kick_gen_approach_estVelCorrection	0.07663597656595554
kick_gen_approach_navBallDist	0.5
kick_gen_approach_navBallCollision	0.333
//kick_gen_approach_navBallAngle	40.575364449933566
kick_gen_approach_navBallAngle	20.0
kick_gen_approach_maxDecelX	1.1764942631380784
kick_gen_approach_maxDecelY	0.5754931880185323


// Parameters for dribbling
drib_coll_thresh				0.33582030312352373
drib_target					0.17785682250297227


)";
#pragma endregion
    wpType3
#pragma region
 = R"(
### Default parameters loaded for all type 3 agents ###

########################
### GETUP PARAMETERS ###
########################

# default getup off front params - optimized for no self collisions
getup_parms_stateDownInitialWait	0.5
getup_parms_stateDown3A1	31.373778672868006
getup_parms_stateDown3L3	132.04992361431232
getup_parms_stateDown3MinTime	0.04
getup_parms_stateDown5L1	-40.19865222799057
getup_parms_stateDown5MinTime	0.9
getup_parms_stateDown7L1	-5.303130226020178
getup_parms_stateDown7L3	61.27667080700868
getup_parms_stateDown7MinTime	0.3
getup_parms_stateDown10MinTime	0.8

# default getup off back params - optimized for no self collisions
getup_parms_stateUpInitialWait	0.5
getup_parms_stateUp3A1	-74.19212693705215
getup_parms_stateUp3A2	22.059688990978437
getup_parms_stateUp3A4	-2.805504697930181
getup_parms_stateUp3L3	28.62816708004984
getup_parms_stateUp3MinTime	0.2
getup_parms_stateUp5L3	150.94279526388357
getup_parms_stateUp5MinTime	0.04
getup_parms_stateUp7L1	-77.42911633035874
getup_parms_stateUp7MinTime	0.2
getup_parms_stateUp9A1	7.430718528871051
getup_parms_stateUp9L1	-97.75479706018714
getup_parms_stateUp9L4	-117.6731135265411
getup_parms_stateUp9L5	-56.61496431999153
getup_parms_stateUp9L6	-12.254544769250895
getup_parms_stateUp9MinTime	0.2
getup_parms_stateUp11A1	-16.438083494272085
getup_parms_stateUp11L1	-140.85938039354576
getup_parms_stateUp11L5	-10.078725855976106
getup_parms_stateUp11MinTime	0.4
getup_parms_stateUp13A1	-134.62269922433833
getup_parms_stateUp13L1	4.809371288194855
getup_parms_stateUp13L3	58.230989205302535
getup_parms_stateUp13L4	-24.26206719200843
getup_parms_stateUp13L5	1.3567977261284696
getup_parms_stateUp13MinTime	0.04 
getup_parms_stateUp15MinTime	0.6


#######################
### KICK PARAMETERS ###
#######################

kick_p1	-50.621274188520516
kick_p2	-96.48124266563154
kick_p3	-66.3674550430713
kick_p4	132.6079562207066
kick_p5	26.422265770856786
kick_p6	57.154201046228906
kick_p7	5.702358695662072
kick_p8	7.877504867837286
kick_p9	11.522796502849335
kick_p11	-36.71509862086306
kick_p12	20.61374682005677
kick_p13	-10.876276156708883
kick_p14	0.7449309633741266
kick_xoffset	-0.2408393951668601
kick_yoffset	-0.06798389074706206
kick_scale1	11.670309819784594
kick_scale2	1.9954055020031927
kick_scale3	-7.5688666112202645


#################################
### OPTIMIZED WALK PARAMETERS ###
#################################

##<--Walk parameters for going to a target-->##
/*
These values are not UT Austin Villa's regular walking to target parameters,
instead the team's parameters for positioning and dribbling are used as a 
placeholder.  See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
utwalk_max_step_size_angle	0.8813862859288328
utwalk_max_step_size_x	107.3345416433581
utwalk_max_step_size_y	50.16358799467425
utwalk_shift_amount	-9.645515001040701
utwalk_walk_height	147.687903533537
utwalk_step_height	74.16613093090523
utwalk_fraction_still	0.3530937822787453
utwalk_fraction_on_ground	-0.09283692479214165
utwalk_phase_length	0.062418067271895444
utwalk_default_com_pos_x	-41.23365401620529
utwalk_pid_step_size_x	0.016096422722122177
utwalk_pid_step_size_y	0.056713579216171206
utwalk_pid_step_size_rot	0.08683126580923822
utwalk_max_normal_com_error	13.88734219175688
utwalk_max_acceptable_com_error	97.61803550155396
utwalk_fwd_offset	2.138608744683571
utwalk_fwd_offset_factor	0.8920279682132857
utwalk_fraction_moving	0.6833323202812207
utwalk_fraction_in_air	1.2966717859105423
utwalk_swing_ankle_offset	-0.011257119451918965
utwalk_pid_tilt	0.1911495121542667
utwalk_pid_roll	-0.32542799483520296
utwalk_pid_com_x	1.3181352799362462
utwalk_pid_com_y	0.6439683323811424
utwalk_pid_com_z	0.10946655507451812
utwalk_pid_arm_x	-0.04252457434628208
utwalk_pid_arm_y	0.19398407014957578

##<--Walk parameters for positioning/dribbling-->##
/*
See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
pos_utwalk_max_step_size_angle	0.8813862859288328
pos_utwalk_max_step_size_x	107.3345416433581
pos_utwalk_max_step_size_y	50.16358799467425
pos_utwalk_shift_amount	-9.645515001040701
pos_utwalk_walk_height	147.687903533537
pos_utwalk_step_height	74.16613093090523
pos_utwalk_fraction_still	0.3530937822787453
pos_utwalk_fraction_on_ground	-0.09283692479214165
pos_utwalk_phase_length	0.062418067271895444
pos_utwalk_default_com_pos_x	-41.23365401620529
pos_utwalk_pid_step_size_x	0.016096422722122177
pos_utwalk_pid_step_size_y	0.056713579216171206
pos_utwalk_pid_step_size_rot	0.08683126580923822
pos_utwalk_max_normal_com_error	13.88734219175688
pos_utwalk_max_acceptable_com_error	97.61803550155396
pos_utwalk_fwd_offset	2.138608744683571
pos_utwalk_fwd_offset_factor	0.8920279682132857
pos_utwalk_fraction_moving	0.6833323202812207
pos_utwalk_fraction_in_air	1.2966717859105423
pos_utwalk_swing_ankle_offset	-0.011257119451918965
pos_utwalk_pid_tilt	0.1911495121542667
pos_utwalk_pid_roll	-0.32542799483520296
pos_utwalk_pid_com_x	1.3181352799362462
pos_utwalk_pid_com_y	0.6439683323811424
pos_utwalk_pid_com_z	0.10946655507451812
pos_utwalk_pid_arm_x	-0.04252457434628208
pos_utwalk_pid_arm_y	0.19398407014957578

##<--Walk parameters for approaching the ball to kick-->##
/*
See the following paper for how these parameters were optimized: 
---
UT Austin Villa: RoboCup 2014 3D Simulation League Competition and Technical Challenge Champions.
Patrick MacAlpine, Mike Depinet, Jason Liang, and Peter Stone.
In RoboCup-2014: Robot Soccer World Cup XVIII, Lecture Notes in Artificial Intelligence, Springer Verlag, Berlin, 2015.
---
*/
app_utwalk_max_step_size_angle	1.5658540891438208
app_utwalk_max_step_size_x	85.62354786632908
app_utwalk_max_step_size_y	66.55493597447787
app_utwalk_shift_amount	-25.717092988455377
app_utwalk_walk_height	155.97699610626586
app_utwalk_step_height	71.91317764128446
app_utwalk_fraction_still	0.5921224357178227
app_utwalk_fraction_on_ground	0.08241881050163216
app_utwalk_phase_length	0.043490336499228746
app_utwalk_default_com_pos_x	-58.004414037674955
app_utwalk_pid_step_size_x	0.06898186727176964
app_utwalk_pid_step_size_y	0.09350623780104678
app_utwalk_pid_step_size_rot	0.09450837962573518
app_utwalk_max_normal_com_error	31.260652541852693
app_utwalk_max_acceptable_com_error	122.96900979391759
app_utwalk_fwd_offset	2.230735167695816
app_utwalk_fwd_offset_factor	0.9254865876701823
app_utwalk_fraction_moving	0.8705953612631807
app_utwalk_fraction_in_air	1.3892580307682738
app_utwalk_swing_ankle_offset	0.08832347939917858
app_utwalk_pid_tilt	0.3048680146143837
app_utwalk_pid_roll	-0.3219664122442003
app_utwalk_pid_com_x	1.435720452879604
app_utwalk_pid_com_y	0.6129731132169935
app_utwalk_pid_com_z	0.03352964983107902
app_utwalk_pid_arm_x	-0.10375221628374033
app_utwalk_pid_arm_y	0.23826286396901017


// Parameters for approaching the ball
kick_gen_approach_turnDist	0.40892318003760225
kick_gen_approach_buff	0.02885478659767846
kick_gen_approach_estVelCorrection	0.09694944837908366
kick_gen_approach_navBallDist	0.5
kick_gen_approach_navBallCollision	0.333
//kick_gen_approach_navBallAngle	40.53921368439362
kick_gen_approach_navBallAngle	20.0
//kick_gen_approach_maxDecelX	0.6496963869244626
kick_gen_approach_maxDecelX	1.0
kick_gen_approach_maxDecelY	1.5491744047740306


// Parameters for dribbling
drib_coll_thresh				0.33582030312352373
drib_target					0.17785682250297227


# t3 walk 10s 7m->8m
/*
utwalk_default_com_pos_x	-39.88466259985527
utwalk_fraction_in_air	1.0968464086774907
utwalk_fraction_moving	0.6421933576054804
utwalk_fraction_on_ground	-0.071169525740414
utwalk_fraction_still	0.38174801371219547
utwalk_fwd_offset	2.078117096864314
utwalk_fwd_offset_factor	1.137646415178301
utwalk_max_acceptable_com_error	97.86709158212308
utwalk_max_normal_com_error	15.863844874071328
utwalk_max_step_size_angle	0.3690384207080618
utwalk_max_step_size_x	107.26300678021114
utwalk_max_step_size_y	51.165576932669275
utwalk_phase_length	0.05234894236856069
utwalk_step_height	75.24452683987863
utwalk_swing_ankle_offset	-0.2476023839729293
utwalk_walk_height	147.81038807299822
*/


# t3 kickik 4.5m 不够稳定,一半的时间不能成功踢球
kick_ik_0_xoffset	-0.14
kick_ik_0_yoffset	-0.03
kick_ik_0_x0	-11.275188000380998
kick_ik_0_y0	-4.765628007009564
kick_ik_0_z0	-7.695919056420388
kick_ik_0_x1	1.820177210346296
kick_ik_0_y1	-17.284880689446236
kick_ik_0_z1	-2.464876036166624
kick_ik_0_x2	-0.6731225924730766
kick_ik_0_y2	12.575844771392306
kick_ik_0_z2	8.787853838808779
kick_ik_0_x3	-10.45747978856282
kick_ik_0_y3	-17.875700721595486
kick_ik_0_z3	-14.433783430873381
kick_ik_0_a0	-10.056264824040653
kick_ik_0_b0	7.980416237529369
kick_ik_0_c0	-17.696717483268134
kick_ik_0_a1	2.492050688721903
kick_ik_0_b1	10.94690687253427
kick_ik_0_c1	3.9775091979954076
kick_ik_0_a2	2.949184896824237
kick_ik_0_b2	1.4827697511352476
kick_ik_0_c2	-13.416116161910848
kick_ik_0_a3	-7.142480079522068
kick_ik_0_b3	103.48458186754786
kick_ik_0_c3	-15.005585404787176
kick_ik_0_wait	-6.360539159062407
kick_ik_0_scale	17.572273969641525
kick_ik_0_off3_0	26.38632426955776
kick_ik_0_off4_0	32.906104338415034
kick_ik_0_off5_0	-27.799855168353687
kick_ik_0_off3_1	-29.763609412263513
kick_ik_0_off4_1	30.1100575266574
kick_ik_0_off5_1	-5.137067888361745
kick_ik_0_off3_2	-59.72327474865031
kick_ik_0_off4_2	-65.66882896153973
kick_ik_0_off5_2	38.65314332913073


# dajiao
dajiao_la0	20
dajiao_ra0	-50
dajiao_kick_j00	-0.4040816314874524
dajiao_kick_j01	-2.1530581599857532
dajiao_kick_j02	0.6544838948839101
dajiao_kick_j03	-0.6923868516499727
dajiao_kick_j04	0.6753531335197651
dajiao_kick_j05	0.5958433099810567
dajiao_kick_j06	2.767114022594593
dajiao_kick_j07	-1.3448803159814338
dajiao_kick_j08	-3.0534975315597017
dajiao_kick_j09	-0.48845977028203413
dajiao_kick_j10	-0.33233902729438547
dajiao_kick_j11	-1.4227770071869246
dajiao_kick_j12	0.3934473936068418
dajiao_kick_j13	-0.322223532663035
dajiao_kick_j14	1.5015996036610897
dajiao_kick_j15	-0.46531180764758384
dajiao_kick_j16	1.6996402842110183
dajiao_kick_j17	1.919402164250752
dajiao_kick_j18	0.33846802909929297
dajiao_kick_j30	-1.372651493971188
dajiao_kick_j31	0.5967445169747537
dajiao_kick_j32	3.1553409518652047
dajiao_kick_j33	-1.430693355404551
dajiao_kick_j34	-0.7244119727857314
dajiao_kick_j40	-0.13548730185381735
dajiao_kick_j41	-1.5906757293714913
dajiao_kick_j42	2.183930455366269
dajiao_kick_j43	-2.2429308582639615
dajiao_kick_j50	6.764151416227097
dajiao_kick_j51	0.9426004915089634
dajiao_kick_j52	3.9864000624307687
dajiao_kick_j53	1.2916667359792433
dajiao_kick_j54	-2.9658711708415546
dajiao_kick_j55	-1.6453344934677314
dajiao_kick_j56	-2.2471495510328885
dajiao_kick_j57	-2.8711602163627084
dajiao_kick_j58	0.6551887869345772
dajiao_kick_j60	-6.406428703688288
dajiao_kick_j61	1.4150368430675342
dajiao_kick_j62	-2.0779901586787766
dajiao_kick_j63	2.144929139733169
dajiao_kick_j64	-3.581619747466207
dajiao_kick_j65	0.805007462190055
dajiao_kick_j66	1.554381107056737
dajiao_kick_j67	-1.0416371627480117
dajiao_kick_j68	0.21483508031289256



)";
#pragma endregion
    wpType4
#pragma region
 = R"(### Default parameters loaded for all type 4 agents ###

########################
### GETUP PARAMETERS ###
########################

# default getup off front params - optimized for no self collisions
getup_parms_stateDownInitialWait	0.5
getup_parms_stateDown3A1	50.98021756053253
getup_parms_stateDown3L3	126.1220933612166
getup_parms_stateDown3MinTime	0.04
getup_parms_stateDown5L1	-48.00207256772667
getup_parms_stateDown5MinTime	0.9
getup_parms_stateDown7L1	-22.340860939025024
getup_parms_stateDown7L3	63.33360057004941
getup_parms_stateDown7MinTime	0.3
getup_parms_stateDown10MinTime	0.8
getup_parms_stateDown3L7	10.434125629408747
getup_parms_stateDown5L7	-24.614475691676574
getup_parms_stateDown7L7	-25.565997236105442

# default getup off back params - optimized for no self collisions
getup_parms_stateUpInitialWait	0.5
getup_parms_stateUp3A1	-160.48868850844917
getup_parms_stateUp3A2	43.87852475409584
getup_parms_stateUp3A4	47.107544084593684
getup_parms_stateUp3L3	15.740604821659796
getup_parms_stateUp3MinTime	0.2
getup_parms_stateUp5L3	132.27670902584242
getup_parms_stateUp5MinTime	0.04
getup_parms_stateUp7L1	-33.46492505628697
getup_parms_stateUp7MinTime	0.2
getup_parms_stateUp9A1	35.11715155456652
getup_parms_stateUp9L1	-93.41203497109993
getup_parms_stateUp9L4	-42.02538707170181
getup_parms_stateUp9L5	-74.20809920914384
getup_parms_stateUp9L6	-45.011664557354386
getup_parms_stateUp9MinTime	0.2
getup_parms_stateUp11A1	12.335079113997592
getup_parms_stateUp11L1	-59.73823689637007
getup_parms_stateUp11L5	-75.30629507279436
getup_parms_stateUp11MinTime	0.4
getup_parms_stateUp13A1	-92.92376016195993
getup_parms_stateUp13L1	-19.862994508032
getup_parms_stateUp13L3	6.715020876796658
getup_parms_stateUp13L4	9.198657450095972
getup_parms_stateUp13L5	-45.49461773853202
getup_parms_stateUp13MinTime	0.04 
getup_parms_stateUp15MinTime	0.6
getup_parms_stateUp3L7	-13.906519868066766
getup_parms_stateUp5L7	-13.153693919511753
getup_parms_stateUp7L7	11.047229348390086
getup_parms_stateUp9L7	-24.75948253132887
getup_parms_stateUp11L7	-47.26657398170224
getup_parms_stateUp13L7	11.057355562206059


#######################
### KICK PARAMETERS ###
#######################

kick_p1	-32.094281611172875
kick_p2	-113.02038630123421
kick_p3	-49.1365943090734
kick_p4	61.30813011871579
kick_p5	-4.6329924876882105
kick_p6	73.59068768840461
kick_p7	20.779448282962882
kick_p8	-42.22550356922419
kick_p9	-2.8401506218779002
kick_p11	0.6189762791149791
kick_p12	-20.81360432334142
kick_p13	-14.693326697129498
kick_p14	3.5631094823005673
kick_xoffset	-0.19897997275908952
kick_yoffset	-0.08550413847290347
kick_scale1	9.801689052827003
kick_scale2	4.424037939123943
kick_scale3	6.549362013596022


#################################
### OPTIMIZED WALK PARAMETERS ###
#################################

##<--Walk parameters for going to a target-->##
/*
These values are not UT Austin Villa's regular walking to target parameters,
instead the team's parameters for positioning and dribbling are used as a 
placeholder.  See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
utwalk_max_step_size_angle	1.4416743787331532
utwalk_max_step_size_x	129.67178389884145
utwalk_max_step_size_y	85.43441476881097
utwalk_shift_amount	-19.072626978466474
utwalk_walk_height	163.95395297542038
utwalk_step_height	69.79970029038759
utwalk_fraction_still	0.40058559009258865
utwalk_fraction_on_ground	-0.07608397347587026
utwalk_phase_length	0.053301855324310365
utwalk_default_com_pos_x	-40.86281474359965
utwalk_pid_step_size_x	0.006198869625225386
utwalk_pid_step_size_y	0.040101946437072676
utwalk_pid_step_size_rot	0.08796723262607366
utwalk_max_normal_com_error	49.639790156128214
utwalk_max_acceptable_com_error	111.3508671336916
utwalk_fwd_offset	4.371051281858148
utwalk_fwd_offset_factor	0.9594908821212534
utwalk_fraction_moving	0.7253595129902992
utwalk_fraction_in_air	1.200986438627392
utwalk_swing_ankle_offset	-0.006622006376079897
utwalk_pid_tilt	0.1479888663812478
utwalk_pid_roll	0.07538683013099545
utwalk_pid_com_x	1.2925753195941794
utwalk_pid_com_y	0.8501332782115112
utwalk_pid_com_z	0.04623210055704251
utwalk_pid_arm_x	0.12364167434370515
utwalk_pid_arm_y	-0.17696608827286658
utwalk_toe_const_offset	0.018836427341286186
utwalk_toe_amplitude	-0.3033274567908883
utwalk_toe_phase_offset	-0.028719513936251818
utwalk_ankle_const_offset	-0.023659361728239663
utwalk_ankle_amplitude	0.08590805524680685
utwalk_ankle_phase_offset	-0.11237480724789313

##<--Walk parameters for positioning/dribbling-->##
/*
See the following paper for how these parameters were optimized: 
---
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
---
*/
pos_utwalk_max_step_size_angle	1.4416743787331532
pos_utwalk_max_step_size_x	129.67178389884145
pos_utwalk_max_step_size_y	85.43441476881097
pos_utwalk_shift_amount	-19.072626978466474
pos_utwalk_walk_height	163.95395297542038
pos_utwalk_step_height	69.79970029038759
pos_utwalk_fraction_still	0.40058559009258865
pos_utwalk_fraction_on_ground	-0.07608397347587026
pos_utwalk_phase_length	0.053301855324310365
pos_utwalk_default_com_pos_x	-40.86281474359965
pos_utwalk_pid_step_size_x	0.006198869625225386
pos_utwalk_pid_step_size_y	0.040101946437072676
pos_utwalk_pid_step_size_rot	0.08796723262607366
pos_utwalk_max_normal_com_error	49.639790156128214
pos_utwalk_max_acceptable_com_error	111.3508671336916
pos_utwalk_fwd_offset	4.371051281858148
pos_utwalk_fwd_offset_factor	0.9594908821212534
pos_utwalk_fraction_moving	0.7253595129902992
pos_utwalk_fraction_in_air	1.200986438627392
pos_utwalk_swing_ankle_offset	-0.006622006376079897
pos_utwalk_pid_tilt	0.1479888663812478
pos_utwalk_pid_roll	0.07538683013099545
pos_utwalk_pid_com_x	1.2925753195941794
pos_utwalk_pid_com_y	0.8501332782115112
pos_utwalk_pid_com_z	0.04623210055704251
pos_utwalk_pid_arm_x	0.12364167434370515
pos_utwalk_pid_arm_y	-0.17696608827286658
pos_utwalk_toe_const_offset	0.018836427341286186
pos_utwalk_toe_amplitude	-0.3033274567908883
pos_utwalk_toe_phase_offset	-0.028719513936251818
pos_utwalk_ankle_const_offset	-0.023659361728239663
pos_utwalk_ankle_amplitude	0.08590805524680685
pos_utwalk_ankle_phase_offset	-0.11237480724789313

##<--Walk parameters for approaching the ball to kick-->##
/*
See the following paper for how these parameters were optimized: 
---
UT Austin Villa: RoboCup 2014 3D Simulation League Competition and Technical Challenge Champions.
Patrick MacAlpine, Mike Depinet, Jason Liang, and Peter Stone.
In RoboCup-2014: Robot Soccer World Cup XVIII, Lecture Notes in Artificial Intelligence, Springer Verlag, Berlin, 2015. 
---
*/
app_utwalk_max_step_size_angle	1.3851241729016985
app_utwalk_max_step_size_x	68.19024604356497
app_utwalk_max_step_size_y	94.55056620399375
app_utwalk_shift_amount	-44.8464852416027
app_utwalk_walk_height	178.22003937598092
app_utwalk_step_height	67.3777982551469
app_utwalk_fraction_still	0.714117565677756
app_utwalk_fraction_on_ground	-0.12027617933178626
app_utwalk_phase_length	0.03402307652014996
app_utwalk_default_com_pos_x	-59.901713024461436
app_utwalk_pid_step_size_x	0.07741715917950512
app_utwalk_pid_step_size_y	0.06352226916148837
app_utwalk_pid_step_size_rot	0.09619610208529214
app_utwalk_max_normal_com_error	70.31992992836732
app_utwalk_max_acceptable_com_error	93.06011810969464
app_utwalk_fwd_offset	2.7592010475737436
app_utwalk_fwd_offset_factor	1.4050901787866963
app_utwalk_fraction_moving	0.5513548149513241
app_utwalk_fraction_in_air	1.5476864942823265
app_utwalk_swing_ankle_offset	-0.06438739941776088
app_utwalk_pid_tilt	0.20278853624453314
app_utwalk_pid_roll	-0.10136767559136398
app_utwalk_pid_com_x	1.0475408001858755
app_utwalk_pid_com_y	1.0407325878750457
app_utwalk_pid_com_z	-0.007113500240289245
app_utwalk_pid_arm_x	0.06943499744229321
app_utwalk_pid_arm_y	-0.17457303939100155
app_utwalk_toe_const_offset	-0.04661057137031381
app_utwalk_toe_amplitude	-0.40329604148288406
app_utwalk_toe_phase_offset	-0.14494086502893036
app_utwalk_ankle_const_offset	-0.030023798435842443
app_utwalk_ankle_amplitude	0.018540560797810964
app_utwalk_ankle_phase_offset	0.0431289060616215


// Parameters for approaching the ball
kick_gen_approach_turnDist	0.44294122666871544
kick_gen_approach_buff	0.2973084057596502
kick_gen_approach_estVelCorrection	0.003118274370542684
kick_gen_approach_navBallDist	0.5
kick_gen_approach_navBallCollision	0.333
//kick_gen_approach_navBallAngle	56.11331429573308
kick_gen_approach_navBallAngle	20.0
kick_gen_approach_maxDecelX	0.9082989933597487
kick_gen_approach_maxDecelY	1.9052608061330103


// Parameters for dribbling
drib_coll_thresh				0.33582030312352373
drib_target					0.17785682250297227)";
#pragma endregion

    kpDajiao
#pragma region
 = R"(
# Time is in seconds.
# Angles are in degrees.
# 改造了大脚动作,添加参数

STARTSKILL SKILL_DAJIAO_LEFT_LEG

# State 0
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 ) EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 0 + $dajiao_kick_j00 ) EFF_LL3 ( 12 + $dajiao_kick_j10 ) EFF_LL4 ( -26 + $dajiao_kick_j30 ) EFF_LL5 20 EFF_LL6 0 EFF_LL7 25   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 0 + $dajiao_kick_j40 ) EFF_RL3 ( 12 + $dajiao_kick_j50 ) EFF_RL4 ( -26 + $dajiao_kick_j60 ) EFF_RL5 20 EFF_RL6 0 EFF_RL7 45   end
wait ( 0.25 + $dajiao_kick_w0 ) end
ENDSTATE

# State 1
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 3 + $dajiao_kick_j01 )  EFF_LL3 ( 12 + $dajiao_kick_j10 ) EFF_LL4 ( -26 + $dajiao_kick_j30 ) EFF_LL5 21  EFF_LL6 -3 EFF_LL7 34   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 3 + $dajiao_kick_j41 ) EFF_RL3 ( 12 + $dajiao_kick_j50 ) EFF_RL4 ( -26 + $dajiao_kick_j60 ) EFF_RL5 21  EFF_RL6 -3 EFF_RL7 47   end
wait ( 0.05 + $dajiao_kick_w1 ) end
ENDSTATE

# State 2
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 6 + $dajiao_kick_j02 )  EFF_LL3 ( 12 + $dajiao_kick_j10 ) EFF_LL4 ( -26 + $dajiao_kick_j30 ) EFF_LL5 22  EFF_LL6 -6 EFF_LL7 43   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 6 + $dajiao_kick_j42 ) EFF_RL3 ( 12 + $dajiao_kick_j50 )  EFF_RL4 ( -26 + $dajiao_kick_j60 ) EFF_RL5 22  EFF_RL6 -6 EFF_RL7 49   end
wait ( 0.05 + $dajiao_kick_w2 ) end
ENDSTATE

# State 3
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 10 + $dajiao_kick_j03 )  EFF_LL3 ( 12 + $dajiao_kick_j10 ) EFF_LL4 ( -26 + $dajiao_kick_j30 ) EFF_LL5 23  EFF_LL6 -10 EFF_LL7 52   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 10 + $dajiao_kick_j43 ) EFF_RL3 ( 12 + $dajiao_kick_j50 ) EFF_RL4 ( -26 + $dajiao_kick_j60 ) EFF_RL5 23  EFF_RL6 -10 EFF_RL7 52   end
wait ( 0.1 + $dajiao_kick_w3 ) end
ENDSTATE

# State 4
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 10 + $dajiao_kick_j04 )  EFF_LL3 ( 12 + $dajiao_kick_j11 ) EFF_LL4 ( -130 + $dajiao_kick_j31 ) EFF_LL5 47  EFF_LL6 -10 EFF_LL7 52   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 10 + $dajiao_kick_j43 ) EFF_RL3 ( 12 + $dajiao_kick_j51 ) EFF_RL4 ( -26 + $dajiao_kick_j61 ) EFF_RL5 23  EFF_RL6 -10 EFF_RL7 52   end
wait ( 0.1 + $dajiao_kick_w4 ) end
ENDSTATE

# State 5
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 8 + $dajiao_kick_j05 )  EFF_LL3 ( 3 + $dajiao_kick_j12 ) EFF_LL4 ( -130 + $dajiao_kick_j31 ) EFF_LL5 28  EFF_LL6 -8 EFF_LL7 52   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 10 + $dajiao_kick_j43 ) EFF_RL3 ( 18 + $dajiao_kick_j52 ) EFF_RL4 ( -20 + $dajiao_kick_j62 ) EFF_RL5 19  EFF_RL6 -10 EFF_RL7 52   end
wait ( 0.05 + $dajiao_kick_w5 ) end
ENDSTATE

# State 6
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 6 + $dajiao_kick_j06 )  EFF_LL3 ( -4 + $dajiao_kick_j13 ) EFF_LL4 ( -130 + $dajiao_kick_j31 ) EFF_LL5 13  EFF_LL6 -6 EFF_LL7 52   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 10 + $dajiao_kick_j43 ) EFF_RL3 ( 22 + $dajiao_kick_j53 ) EFF_RL4 ( -15 + $dajiao_kick_j63 ) EFF_RL5 16  EFF_RL6 -10 EFF_RL7 52   end
wait ( 0.05 + $dajiao_kick_w6 ) end
ENDSTATE

# State 7
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 4 + $dajiao_kick_j07 )  EFF_LL3 ( -11 + $dajiao_kick_j14 ) EFF_LL4 ( -130 + $dajiao_kick_j31 ) EFF_LL5 0  EFF_LL6 -4 EFF_LL7 52   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 10 + $dajiao_kick_j43 ) EFF_RL3 ( 26 + $dajiao_kick_j54 ) EFF_RL4 ( -11 + $dajiao_kick_j64 ) EFF_RL5 13  EFF_RL6 -10 EFF_RL7 52   end
wait ( 0.05 + $dajiao_kick_w7 ) end
ENDSTATE

# State 8
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 2 + $dajiao_kick_j08 )  EFF_LL3 ( -18 + $dajiao_kick_j15 ) EFF_LL4 ( -130 + $dajiao_kick_j31 ) EFF_LL5 -14  EFF_LL6 -2 EFF_LL7 52   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 10 + $dajiao_kick_j43 ) EFF_RL3 ( 30 + $dajiao_kick_j55 ) EFF_RL4 ( -7 + $dajiao_kick_j65 ) EFF_RL5 10  EFF_RL6 -10 EFF_RL7 52   end
wait ( 0.05 + $dajiao_kick_w8 ) end
ENDSTATE

# State 9
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 0 + $dajiao_kick_j09 )  EFF_LL3 ( -25 + $dajiao_kick_j16 ) EFF_LL4 ( -130 + $dajiao_kick_j32 ) EFF_LL5 -29  EFF_LL6 0 EFF_LL7 52   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 10 + $dajiao_kick_j43 ) EFF_RL3 ( 35 + $dajiao_kick_j56 ) EFF_RL4 ( -3 + $dajiao_kick_j66 ) EFF_RL5 7  EFF_RL6 -10 EFF_RL7 52   end
wait ( 0.2 + $dajiao_kick_w9 ) end
ENDSTATE

# State 10
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 0 + $dajiao_kick_j09 )  EFF_LL3 ( -25 + $dajiao_kick_j17 ) EFF_LL4 ( -130 + $dajiao_kick_j33 ) EFF_LL5 -29  EFF_LL6 0 EFF_LL7 25   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 10 + $dajiao_kick_j43 ) EFF_RL3 ( 45 + $dajiao_kick_j57 ) EFF_RL4 ( 1 + $dajiao_kick_j67 ) EFF_RL5 -37  EFF_RL6 -10 EFF_RL7 45   end
wait ( 0.16 + $dajiao_kick_w10 ) end
ENDSTATE

# State 11
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 ( 0 + $dajiao_la0 )  EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 ( 0 + $dajiao_kick_j09 )  EFF_LL3 ( 100 + $dajiao_kick_j18 ) EFF_LL4 ( -19 + $dajiao_kick_j34 ) EFF_LL5 -29  EFF_LL6 0 EFF_LL7 25   end
settar EFF_RA1 -84 EFF_RA2 ( 0 + $dajiao_ra0 ) EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 ( 10 + $dajiao_kick_j43 ) EFF_RL3 ( -25 + $dajiao_kick_j58 ) EFF_RL4 ( -130+ $dajiao_kick_j68 ) EFF_RL5 -37  EFF_RL6 -10 EFF_RL7 45   end
wait ( 0.6 + $dajiao_kick_w11 ) end
ENDSTATE

ENDSKILL

REFLECTSKILL SKILL_DAJIAO_LEFT_LEG SKILL_DAJIAO_RIGHT_LEG


)";
#pragma endregion


    
    kpBigkick
#pragma region
 = R"(# Time is in seconds.
# Angles are in degrees.

STARTSKILL SKILL_BIG_KICK_RIGHT_LEG

# State 0
STARTSTATE
settar EFF_LL1 0 EFF_LL2 0 EFF_LL3 20 EFF_LL4 -30 EFF_LL5 20 EFF_LL6 0 end
settar EFF_RL1 0 EFF_RL2 0 EFF_RL3 20 EFF_RL4 -30 EFF_RL5 15 EFF_RL6 0 end
wait 0.1 end
ENDSTATE



# State 1
STARTSTATE
settar EFF_LL1 0 EFF_LL2 10 EFF_LL3 20 EFF_LL4 -30 EFF_LL5 20 EFF_LL6 -10 end
settar EFF_RL1 0 EFF_RL2 10 EFF_RL3 20 EFF_RL4 -30 EFF_RL5 20 EFF_RL6 -10 end
wait 0.2 end
ENDSTATE

# State 2
STARTSTATE
settar EFF_LL1 0 EFF_LL2 10 EFF_LL3 20 EFF_LL4 -45 EFF_LL5 35 EFF_LL6 -10 end
settar EFF_RL1 0 EFF_RL2 10 EFF_RL3 20 EFF_RL4 -30 EFF_RL5 20 EFF_RL6 -10 end
wait 0.2 end
ENDSTATE

# State 3
STARTSTATE
settar EFF_LL1 0 EFF_LL2 10 EFF_LL3 40 EFF_LL4 -40 EFF_LL5 10 EFF_LL6 -10 end
settar EFF_RL1 0 EFF_RL2 10 EFF_RL3 20 EFF_RL4 -30 EFF_RL5 20 EFF_RL6 -10 end
wait 0.2 end
ENDSTATE 

# State 4
STARTSTATE
settar EFF_LL1 0 EFF_LL2 10 EFF_LL3 40 EFF_LL4 -40 EFF_LL5 10 EFF_LL6 -10 end
settar EFF_RL1 0 EFF_RL2 10 EFF_RL3 15 EFF_RL4 -25 EFF_RL5 20 EFF_RL6 -10 end
wait 0.2 end
ENDSTATE

# State 5
STARTSTATE
settar EFF_LL1 0 EFF_LL2 ( 0 + $big_kick_p1 ) EFF_LL3 60 EFF_LL4 -45 EFF_LL5 10 EFF_LL6 ( 0 + $big_kick_p2 ) end
settar EFF_RL1 0 EFF_RL2 ( 0 + $big_kick_p1 ) EFF_RL3 10 EFF_RL4 -25 EFF_RL5 40 EFF_RL6 ( 0 + $big_kick_p2 ) end
wait 0.2 end
ENDSTATE

# State 6
STARTSTATE
settar EFF_LL1 0 EFF_LL2 ( -10 + $big_kick_p3 ) EFF_LL3 40 EFF_LL4 -45 EFF_LL5 20 EFF_LL6 ( 10 + $big_kick_p4 ) end
settar EFF_RL1 0 EFF_RL2 ( -10 + $big_kick_p3 ) EFF_RL3 5 EFF_RL4 -25 EFF_RL5 35 EFF_RL6 ( 10 + $big_kick_p4 ) end
wait 0.8 end
ENDSTATE

# State 7
STARTSTATE
settar EFF_LL1 0 EFF_LL2 ( -10 + $big_kick_p3 ) EFF_LL3 40 EFF_LL4 -45 EFF_LL5 20 EFF_LL6 ( 10 + $big_kick_p4 ) end
settar EFF_RL1 0 EFF_RL2 ( -10 + $big_kick_p3 ) EFF_RL3 5 EFF_RL4 -40 EFF_RL5 50 EFF_RL6 ( 10 + $big_kick_p4 ) EFF_RL7 ( 0 + $big_kick_t1 ) end
wait 0.2 end
ENDSTATE

# State 8
STARTSTATE
settar EFF_LL1 0 EFF_LL2 ( -10 + $big_kick_p3 ) EFF_LL3 40 EFF_LL4 -45 EFF_LL5 20 EFF_LL6 ( 10 + $big_kick_p4 ) end
settar EFF_RL1 0 EFF_RL2 ( -10 + $big_kick_p3 ) EFF_RL3 -25 EFF_RL4 -130 EFF_RL5 75 EFF_RL6 ( 10 + $big_kick_p4 ) EFF_RL7 ( 0 + $big_kick_t2 ) end
wait 0.4 end
ENDSTATE

# State 9
STARTSTATE
settar EFF_LL1 0 EFF_LL2 ( -10 + $big_kick_p3 ) EFF_LL3 30 EFF_LL4 -45 EFF_LL5 20 EFF_LL6 ( 10 + $big_kick_p4 ) end
settar EFF_RL1 0 EFF_RL2 -10 EFF_RL3 20 EFF_RL4 -130 EFF_RL5 -42 EFF_RL6 0  EFF_RL7 ( 0 + $big_kick_t3 ) end
wait 0.4 end
ENDSTATE

# State 10
STARTSTATE
settar EFF_LL1 0 EFF_LL2 ( -10 + $big_kick_n1 ) EFF_LL3 -25 EFF_LL4 -130 EFF_LL5 -15 EFF_LL6 ( 10 + $big_kick_p4 ) end
settar EFF_RL1 0 EFF_RL2 -10 EFF_RL3 100 EFF_RL4 1 EFF_RL5 -42 EFF_RL6 0 EFF_RL7 ( 0 + $big_kick_t4 ) end
wait 0.3 end
ENDSTATE

# State 11
STARTSTATE
settar EFF_LL1 0 EFF_LL2 ( -10 + $big_kick_n1 ) EFF_LL3 -25 EFF_LL4 -130 EFF_LL5 -15 EFF_LL6 ( 10 + $big_kick_p4 ) end
settar EFF_RL1 0 EFF_RL2 -10 EFF_RL3 100 EFF_RL4 1 EFF_RL5 75 EFF_RL6 0 EFF_RL7 ( 0 + $big_kick_t4 ) end
wait 0.3 end
ENDSTATE


ENDSKILL

REFLECTSKILL SKILL_BIG_KICK_RIGHT_LEG SKILL_BIG_KICK_LEFT_LEG

)";
#pragma endregion
    kpKick0
#pragma region
 = R"(# Time is in seconds.
# Angles are in degrees.

# BALL IN FRONT OF PLAYER. Kicks forward
# Assumes default position relative to ball around -- x = .2 y = .05

STARTSKILL SKILL_KICK_IK_0_LEFT_LEG
# State 0
STARTSTATE
settar EFF_LL1 0 EFF_LL2 0 EFF_LL3 10.0 EFF_LL4 -20.0 EFF_LL5 10.0 EFF_LL6 0 end
settar EFF_RL1 0 EFF_RL2 0 EFF_RL3 10.0 EFF_RL4 -20.0 EFF_RL5 10.0 EFF_RL6 0 end

settar EFF_RA2 -45 end
settar EFF_LA2 -45 end

settar EFF_RL2 5.0 EFF_RL6 -5.0 end
settar EFF_LL2 5.0 EFF_LL6 -5.0 end

settar EFF_RL2 12.0 EFF_RL6 -12.0 end
settar EFF_LL2 10.0 EFF_LL6 -10.0 end

inctar EFF_LL3 42.0 EFF_LL4 -42.0 EFF_LL5 12.0 end
inctar EFF_RL3 35.0 EFF_RL4 -35.0 EFF_RL5 10.0 end

inctar EFF_RL3 $kick_ik_0_off3_0 EFF_RL4 $kick_ik_0_off4_0 EFF_RL5 $kick_ik_0_off5_0 end

wait 0.15 end
ENDSTATE
# Done shifting weight.

# State 10
STARTSTATE
inctar EFF_RL3 $kick_ik_0_off3_1 EFF_RL4 $kick_ik_0_off4_1 EFF_RL5 $kick_ik_0_off5_1 end

# Move the foot to a preset location w.r.t. the ball
setfoot LEG_LEFT ($kick_ik_0_x0) ($kick_ik_0_y0) ($kick_ik_0_z0) ($kick_ik_0_a0) ($kick_ik_0_b0) ($kick_ik_0_c0) end
wait .1 end
ENDSTATE

# B-Spline Curve
STARTSTATE
inctar EFF_RL3 $kick_ik_0_off3_2 EFF_RL4 $kick_ik_0_off4_2 EFF_RL5 $kick_ik_0_off5_2 end

setscale EFF_LL2 $kick_ik_0_scale EFF_LL3 $kick_ik_0_scale EFF_LL4 $kick_ik_0_scale end
STARTCURVE LEG_LEFT
controlpoint ($kick_ik_0_x0) ($kick_ik_0_y0) ($kick_ik_0_z0) ($kick_ik_0_a0) ($kick_ik_0_b0) ($kick_ik_0_c0) end
# swing leg back
#controlpoint ($kick_ik_0_x1) ($kick_ik_0_y1) ($kick_ik_0_z1) ($kick_ik_0_a1) ($kick_ik_0_b1) ($kick_ik_0_c1) end
# start swinging leg forward towards ball
controlpoint ($kick_ik_0_x2) ($kick_ik_0_y2) ($kick_ik_0_z2) ($kick_ik_0_a2) ($kick_ik_0_b2) ($kick_ik_0_c2) end
# swing leg through ball
controlpoint ($kick_ik_0_x3) ($kick_ik_0_y3) ($kick_ik_0_z3) ($kick_ik_0_a3) ($kick_ik_0_b3) ($kick_ik_0_c3) end
ENDCURVE
wait ($kick_ik_0_wait) end
ENDSTATE

STARTSTATE
wait .1 end
ENDSTATE

STARTSTATE
setscale EFF_LL2 1 EFF_LL3 1 EFF_LL4 1 end
reset LEG_LEFT LEG_RIGHT end
wait .1 end
ENDSTATE

ENDSKILL 

REFLECTSKILL SKILL_KICK_IK_0_LEFT_LEG SKILL_KICK_IK_0_RIGHT_LEG
)";
#pragma endregion
    kpKickik0
#pragma region
 = R"(# Time is in seconds.
# Angles are in degrees.

# BALL IN FRONT OF PLAYER. Kicks forward
# Assumes default position relative to ball around -- x = .2 y = .05

STARTSKILL SKILL_KICK_IK_0_LEFT_LEG
# State 0
STARTSTATE
settar EFF_LL1 0 EFF_LL2 0 EFF_LL3 10.0 EFF_LL4 -20.0 EFF_LL5 10.0 EFF_LL6 0 end
settar EFF_RL1 0 EFF_RL2 0 EFF_RL3 10.0 EFF_RL4 -20.0 EFF_RL5 10.0 EFF_RL6 0 end

settar EFF_RA2 -45 end
settar EFF_LA2 -45 end

settar EFF_RL2 5.0 EFF_RL6 -5.0 end
settar EFF_LL2 5.0 EFF_LL6 -5.0 end

settar EFF_RL2 12.0 EFF_RL6 -12.0 end
settar EFF_LL2 10.0 EFF_LL6 -10.0 end

inctar EFF_LL3 42.0 EFF_LL4 -42.0 EFF_LL5 12.0 end
inctar EFF_RL3 35.0 EFF_RL4 -35.0 EFF_RL5 10.0 end

inctar EFF_RL3 $kick_ik_0_off3_0 EFF_RL4 $kick_ik_0_off4_0 EFF_RL5 $kick_ik_0_off5_0 end

wait 0.15 end
ENDSTATE
# Done shifting weight.

# State 10
STARTSTATE
inctar EFF_RL3 $kick_ik_0_off3_1 EFF_RL4 $kick_ik_0_off4_1 EFF_RL5 $kick_ik_0_off5_1 end

# Move the foot to a preset location w.r.t. the ball
setfoot LEG_LEFT ($kick_ik_0_x0) ($kick_ik_0_y0) ($kick_ik_0_z0) ($kick_ik_0_a0) ($kick_ik_0_b0) ($kick_ik_0_c0) end
wait .1 end
ENDSTATE

# B-Spline Curve
STARTSTATE
inctar EFF_RL3 $kick_ik_0_off3_2 EFF_RL4 $kick_ik_0_off4_2 EFF_RL5 $kick_ik_0_off5_2 end

setscale EFF_LL2 $kick_ik_0_scale EFF_LL3 $kick_ik_0_scale EFF_LL4 $kick_ik_0_scale end
STARTCURVE LEG_LEFT
controlpoint ($kick_ik_0_x0) ($kick_ik_0_y0) ($kick_ik_0_z0) ($kick_ik_0_a0) ($kick_ik_0_b0) ($kick_ik_0_c0) end
# swing leg back
#controlpoint ($kick_ik_0_x1) ($kick_ik_0_y1) ($kick_ik_0_z1) ($kick_ik_0_a1) ($kick_ik_0_b1) ($kick_ik_0_c1) end
# start swinging leg forward towards ball
controlpoint ($kick_ik_0_x2) ($kick_ik_0_y2) ($kick_ik_0_z2) ($kick_ik_0_a2) ($kick_ik_0_b2) ($kick_ik_0_c2) end
# swing leg through ball
controlpoint ($kick_ik_0_x3) ($kick_ik_0_y3) ($kick_ik_0_z3) ($kick_ik_0_a3) ($kick_ik_0_b3) ($kick_ik_0_c3) end
ENDCURVE
wait ($kick_ik_0_wait) end
ENDSTATE

STARTSTATE
wait .1 end
ENDSTATE

STARTSTATE
setscale EFF_LL2 1 EFF_LL3 1 EFF_LL4 1 end
reset LEG_LEFT LEG_RIGHT end
wait .1 end
ENDSTATE

ENDSKILL 

REFLECTSKILL SKILL_KICK_IK_0_LEFT_LEG SKILL_KICK_IK_0_RIGHT_LEG
)";
#pragma endregion

    kpKick
#pragma region
 = R"(# Time is in seconds.
# Angles are in degrees.

STARTSKILL SKILL_KICK_LEFT_LEG

# State 0
STARTSTATE
settar EFF_LL1 0 EFF_LL2 0 EFF_LL3 18.0 EFF_LL4 -30.0 EFF_LL5 18.0 EFF_LL6 0 end
settar EFF_RL1 0 EFF_RL2 0 EFF_RL3 18.0 EFF_RL4 -30.0 EFF_RL5 18.0 EFF_RL6 0 end
wait 0.3 end
ENDSTATE

# State 1 
STARTSTATE
settar EFF_LL2 -5 EFF_LL6 5 EFF_RL2 -5 EFF_RL6 5 end
wait 0.1 end
ENDSTATE

# State 2 
STARTSTATE
settar EFF_LL2 -10 EFF_LL6 10 EFF_RL2 -10 EFF_RL6 10 end
wait 0.1 end
ENDSTATE

# State 3 
STARTSTATE
inctar EFF_RL3 10 EFF_RL4 -10 end
wait 0.1 end
ENDSTATE

# State 4 
STARTSTATE
inctar EFF_RL4 10 EFF_RL5 -10 end
settar EFF_RL1 -5.0 end
wait 0.1 end
ENDSTATE

# State 5 
STARTSTATE
inctar EFF_LL4 -5 EFF_LL5 5 end
wait 0.06 end
ENDSTATE

# State 6 
STARTSTATE
settar EFF_LL2 -6 EFF_LL6 6 EFF_RL2 -6 EFF_RL6 6 end
wait 0.06 end
ENDSTATE

# State 7 
STARTSTATE
settar EFF_LL2 0 EFF_LL6 0 EFF_RL2 0 EFF_RL6 0 end
wait 0.06 end
ENDSTATE

# State 8 
STARTSTATE
settar EFF_LL2 6 EFF_LL6 -6 EFF_RL2 6 EFF_RL6 -6 end
wait 0.06 end
ENDSTATE

# State 9 
STARTSTATE
settar EFF_LL2 ( 12 + $kick_p13 ) EFF_LL6 ( -12 + $kick_p14 ) end
settar EFF_RL2 ( 12 + $kick_p13 ) EFF_RL6 ( -12 + $kick_p14 ) end
wait 0.4 end
ENDSTATE

# State 10 
STARTSTATE
inctar EFF_LL3 -10 EFF_LL4 10 EFF_RL3 -10 EFF_RL4 10 end
wait 0.2 end
ENDSTATE

# State 11 
STARTSTATE
inctar EFF_LL3 -10 EFF_LL4 10 EFF_RL3 -10 EFF_RL4 10 end
wait 0.2 end
ENDSTATE

# State 13 
STARTSTATE
inctar EFF_LL3 20 EFF_LL4 -60 EFF_LL5 20 end
wait 0.2 end
ENDSTATE

# State 14 
STARTSTATE
settar EFF_LL3 ( 50 + $kick_p1 ) EFF_LL4 ( -95 + $kick_p2 ) EFF_LL5 ( -15 + $kick_p3 ) end
wait 0.5 end
ENDSTATE

# State 15 
STARTSTATE
setscale EFF_LL3 ( 3 + $kick_scale1 ) EFF_LL4 ( 3 + $kick_scale2 ) EFF_LL5 ( 3 + $kick_scale3 ) end
settar EFF_LL3 ( 90 + $kick_p4 ) EFF_LL4 ( 0 + $kick_p5 ) EFF_LL5 ( -15 + $kick_p6 ) end
wait 0.5 end
ENDSTATE

# State 16 
STARTSTATE
setscale EFF_LL3 1 EFF_LL4 1 EFF_LL5 1 end
settar EFF_LL3 30 EFF_LL4 -20 EFF_LL5 -10 end
wait 0.1 end
ENDSTATE

# State 17
STARTSTATE
reset LEG_LEFT LEG_RIGHT end
wait 0.8 end
ENDSTATE


ENDSKILL 

REFLECTSKILL SKILL_KICK_LEFT_LEG SKILL_KICK_RIGHT_LEG

)";
#pragma endregion
    kpLeftblock
#pragma region
 = R"(STARTSKILL SKILL_LEFT_BLOCK

# State 0
STARTSTATE
settar EFF_LA1 -40 EFF_LA2 -1 EFF_LA3 0 EFF_LA4 0 EFF_LL1 -70 EFF_LL2 36 EFF_LL3 -4 EFF_LL4 -5 EFF_LL5 9 EFF_LL6 -45 EFF_LL7 0   end
settar EFF_RA1 -120 EFF_RA2 -30 EFF_RA3 0 EFF_RA4 0 EFF_RL1 -33 EFF_RL2 -45 EFF_RL3 100 EFF_RL4 -130 EFF_RL5 72 EFF_RL6 39 EFF_RL7 0   end
wait 0.25 end
ENDSTATE

# State 1 
STARTSTATE
settar EFF_LA1 60 EFF_LA2 4  EFF_LA3 -2 EFF_LA4 -13 EFF_LL1 1 EFF_LL2 9  EFF_LL3 -25 EFF_LL4 -5 EFF_LL5 -45  EFF_LL6 -45 EFF_LL7 0   end
settar EFF_RA1 -114 EFF_RA2 -27 EFF_RA3 93 EFF_RA4 3 EFF_RL1 -15 EFF_RL2 -9 EFF_RL3 -25 EFF_RL4 -22 EFF_RL5 12  EFF_RL6 45 EFF_RL7 0   end
wait 1 end
ENDSTATE

ENDSKILL
)";
#pragma endregion
    kpRightblock
#pragma region
 = R"(STARTSKILL SKILL_RIGHT_BLOCK

# State 0
STARTSTATE
settar EFF_LA1 -120 EFF_LA2 30 EFF_LA3 0 EFF_LA4 0 EFF_LL1 -33 EFF_LL2 45 EFF_LL3 100 EFF_LL4 -130 EFF_LL5 72 EFF_LL6 -39 EFF_LL7 0   end
settar EFF_RA1 -40 EFF_RA2 1 EFF_RA3 0 EFF_RA4 0 EFF_RL1 -70 EFF_RL2 -36 EFF_RL3 -4 EFF_RL4 -5 EFF_RL5 9 EFF_RL6 45 EFF_RL7 0   end
wait 0.25 end
ENDSTATE

# State 1 
STARTSTATE
settar EFF_LA1 -114 EFF_LA2 27  EFF_LA3 -93 EFF_LA4 -3 EFF_LL1 -15 EFF_LL2 9  EFF_LL3 -25 EFF_LL4 -22 EFF_LL5 -12  EFF_LL6 -45 EFF_LL7 0   end
settar EFF_RA1 60 EFF_RA2 -4 EFF_RA3 2 EFF_RA4 13 EFF_RL1 1 EFF_RL2 -9 EFF_RL3 -25 EFF_RL4 -5 EFF_RL5 -45  EFF_RL6 45 EFF_RL7 0   end
wait 1 end
ENDSTATE

ENDSKILL
)";
#pragma endregion
    kpMiddleBlock
#pragma region
 = R"(STARTSKILL SKILL_MIDDLE_BLOCK

# State 0
STARTSTATE
settar EFF_LA1 0 EFF_LA2 60 EFF_LA3 -60 EFF_LA4 -40 EFF_LL1 -90 EFF_LL2 0 EFF_LL3 75 EFF_LL4 0 EFF_LL5 0 EFF_LL6 0 EFF_LL7 0   end
settar EFF_RA1 0 EFF_RA2 -60 EFF_RA3 60 EFF_RA4 40 EFF_RL1 -90 EFF_RL2 -20 EFF_RL3 75 EFF_RL4 0 EFF_RL5 0 EFF_RL6 0 EFF_RL7 0   end
wait 1 end
ENDSTATE

ENDSKILL
)";
#pragma endregion
    kpLongpass
#pragma region
 = R"(# Time is in seconds.
# Angles are in degrees.

STARTSKILL SKILL_LONGASS_LEFT_LEG


# State 0
STARTSTATE
settar EFF_LA1 -44 EFF_LA2 0 EFF_LA3 0 EFF_LA4 0 EFF_LL1 0 EFF_LL2 0 EFF_LL3 12 EFF_LL4 -26 EFF_LL5 20 EFF_LL6 0 EFF_LL7 0   end
settar EFF_RA1 -44 EFF_RA2 0 EFF_RA3 0 EFF_RA4 0 EFF_RL1 0 EFF_RL2 0 EFF_RL3 12 EFF_RL4 -26 EFF_RL5 20 EFF_RL6 0 EFF_RL7 0   end
wait $longpass_time0 end
ENDSTATE

# State 1
STARTSTATE
settar EFF_LA1 -44 EFF_LA2 0 EFF_LA3 0 EFF_LA4 0 EFF_LL1 0 EFF_LL2 10 EFF_LL3 12 EFF_LL4 -26 EFF_LL5 23 EFF_LL6 -10 EFF_LL7 0   end
settar EFF_RA1 -44 EFF_RA2 0 EFF_RA3 0 EFF_RA4 0 EFF_RL1 0 EFF_RL2 10 EFF_RL3 12 EFF_RL4 -26 EFF_RL5 23 EFF_RL6 -10 EFF_RL7 0   end
wait $longpass_time1 end
ENDSTATE

# State 2
STARTSTATE
settar EFF_LA1 -44 EFF_LA2 0 EFF_LA3 0 EFF_LA4 0 EFF_LL1 0 EFF_LL2 10 EFF_LL3 12 EFF_LL4 -130 EFF_LL5 47 EFF_LL6 -10 EFF_LL7 52   end
settar EFF_RA1 -44 EFF_RA2 0 EFF_RA3 0 EFF_RA4 0 EFF_RL1 0 EFF_RL2 10 EFF_RL3 12 EFF_RL4 -26 EFF_RL5 23 EFF_RL6 -10 EFF_RL7 0   end
wait $longpass_time2 end
ENDSTATE

# State 3
STARTSTATE
settar EFF_LA1 -44 EFF_LA2 0 EFF_LA3 0 EFF_LA4 0 EFF_LL1 0 EFF_LL2 0 EFF_LL3 0 EFF_LL4 -121 EFF_LL5 -9 EFF_LL6 0 EFF_LL7 34   end
settar EFF_RA1 -44 EFF_RA2 0 EFF_RA3 0 EFF_RA4 0 EFF_RL1 0 EFF_RL2 10 EFF_RL3 17 EFF_RL4 -3 EFF_RL5 7 EFF_RL6 -10 EFF_RL7 0   end
wait $longpass_time3 end
ENDSTATE

# State 4
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 0 EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 0 EFF_LL3 100 EFF_LL4 -19 EFF_LL5 -29 EFF_LL6 0 EFF_LL7 15   end
settar EFF_RA1 -84 EFF_RA2 0 EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 10 EFF_RL3 -25 EFF_RL4 -130 EFF_RL5 -37 EFF_RL6 -10 EFF_RL7 0   end
wait $longpass_time4 end
ENDSTATE

ENDSKILL

REFLECTSKILL SKILL_LONGASS_LEFT_LEG SKILL_LONGASS_RIGHT_LEG

)";
#pragma endregion
    kpSlowpass
#pragma region
 = R"(# Time is in seconds.
# Angles are in degrees.

STARTSKILL SKILL_SLOWASS_LEFT_LEG

# State 0
STARTSTATE
settar EFF_LA1 -44 EFF_LA2 0 EFF_LA3 0 EFF_LA4 0 EFF_LL1 0 EFF_LL2 0 EFF_LL3 20 EFF_LL4 -30 EFF_LL5 20 EFF_LL6 0 EFF_LL7 0   end
settar EFF_RA1 -44 EFF_RA2 0 EFF_RA3 0 EFF_RA4 0 EFF_RL1 0 EFF_RL2 0 EFF_RL3 20 EFF_RL4 -30 EFF_RL5 20 EFF_RL6 0 EFF_RL7 0   end
wait 0.2 end
ENDSTATE

# State 1
STARTSTATE
settar EFF_LA1 -44 EFF_LA2 0 EFF_LA3 0 EFF_LA4 0 EFF_LL1 0 EFF_LL2 0 EFF_LL3 12 EFF_LL4 -26 EFF_LL5 20 EFF_LL6 0 EFF_LL7 0   end
settar EFF_RA1 -44 EFF_RA2 0 EFF_RA3 0 EFF_RA4 0 EFF_RL1 0 EFF_RL2 0 EFF_RL3 12 EFF_RL4 -26 EFF_RL5 20 EFF_RL6 0 EFF_RL7 0   end
wait 0.6 end
ENDSTATE

# State 2
STARTSTATE
settar EFF_LA1 -44 EFF_LA2 0 EFF_LA3 0 EFF_LA4 0 EFF_LL1 0 EFF_LL2 10 EFF_LL3 12 EFF_LL4 -26 EFF_LL5 23 EFF_LL6 -10 EFF_LL7 0   end
settar EFF_RA1 -44 EFF_RA2 0 EFF_RA3 0 EFF_RA4 0 EFF_RL1 0 EFF_RL2 10 EFF_RL3 12 EFF_RL4 -26 EFF_RL5 23 EFF_RL6 -10 EFF_RL7 0   end
wait 0.4 end
ENDSTATE

# State 3
STARTSTATE
settar EFF_LA1 -44 EFF_LA2 0 EFF_LA3 0 EFF_LA4 0 EFF_LL1 0 EFF_LL2 10 EFF_LL3 12 EFF_LL4 -130 EFF_LL5 47 EFF_LL6 -10 EFF_LL7 52   end
settar EFF_RA1 -44 EFF_RA2 0 EFF_RA3 0 EFF_RA4 0 EFF_RL1 0 EFF_RL2 10 EFF_RL3 12 EFF_RL4 -26 EFF_RL5 23 EFF_RL6 -10 EFF_RL7 0   end
wait 0.3 end
ENDSTATE

# State 4
STARTSTATE
settar EFF_LA1 -44 EFF_LA2 0 EFF_LA3 0 EFF_LA4 0 EFF_LL1 0 EFF_LL2 0 EFF_LL3 0 EFF_LL4 -121 EFF_LL5 -9 EFF_LL6 0 EFF_LL7 34   end
settar EFF_RA1 -44 EFF_RA2 0 EFF_RA3 0 EFF_RA4 0 EFF_RL1 0 EFF_RL2 10 EFF_RL3 17 EFF_RL4 -3 EFF_RL5 7 EFF_RL6 -10 EFF_RL7 0   end
wait 0.4 end
ENDSTATE

# State 5
STARTSTATE
settar EFF_LA1 -84 EFF_LA2 0 EFF_LA3 -67 EFF_LA4 -52 EFF_LL1 0 EFF_LL2 0 EFF_LL3 100 EFF_LL4 -19 EFF_LL5 -29 EFF_LL6 0 EFF_LL7 15   end
settar EFF_RA1 -84 EFF_RA2 0 EFF_RA3 67 EFF_RA4 52 EFF_RL1 0 EFF_RL2 10 EFF_RL3 -25 EFF_RL4 -130 EFF_RL5 -37 EFF_RL6 -10 EFF_RL7 0   end
wait 0.4 end
ENDSTATE

ENDSKILL

REFLECTSKILL SKILL_SLOWASS_LEFT_LEG SKILL_SLOWASS_RIGHT_LEG
)";
#pragma endregion
    kpStand
#pragma region
 = R"(STARTSKILL SKILL_STAND

STARTSTATE
reset ARM_LEFT ARM_RIGHT LEG_LEFT LEG_RIGHT end
inctar EFF_LL3 19 EFF_LL4 -40 EFF_LL5 20 EFF_RL3 19 EFF_RL4 -40 EFF_RL5 20 end
#wait 0.2 end
#wait 0.5 end
wait 0.04 end
ENDSTATE

ENDSKILL 
)";
#pragma endregion
}