#include "naobehavior.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cctype>
#include <exception>

#include "../skills/skillparser.h"
#include "../rvdraw/rvdraw.h"
#include <assert.h>

// For UT Walk
#include <common/InterfaceInfo.h>
#include <motion/MotionModule.h>
#include <../params/params.hpp>
extern int agentBodyType;
extern bool read_paramfiles;
extern void PutMessage(const string& msg);

/*
 * namedParams_ are a mapping between parameters and their values
 */
NaoBehavior::
NaoBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_) :
    namedParams( namedParams_ ),
    rsg( rsg_ )
{

    //cout << "Constructing of Nao Behavior" << endl;

    srand ((unsigned)time(NULL) );
    srand48((unsigned)time(NULL));

    classname = "NaoBehavior"; //TODO: eliminate it...

    mInit = false;
    initBeamed = false;

    agentTeamName = teamName;
    agentUNum = uNum;

    scoreMe = 0;
    scoreOpp = 0;

    worldModel = new WorldModel();
    bodyModel = new BodyModel(worldModel);

    memory_ = new Memory(false,true);

    memory_->getOrAddBlockByName(frame_info_,"frame_info");
    memory_->getOrAddBlockByName(vision_frame_info_,"vision_frame_info");
    frame_info_->source = MEMORY_SIM; // set to simulaor
    vision_frame_info_->source = MEMORY_SIM;

    memory_->getOrAddBlockByName(raw_sensors_,"raw_sensors");
    memory_->getOrAddBlockByName(raw_joint_angles_,"raw_joint_angles");
    memory_->getOrAddBlockByName(processed_joint_angles_,"processed_joint_angles");
    memory_->getOrAddBlockByName(raw_joint_commands_,"raw_joint_commands");
    memory_->getOrAddBlockByName(processed_joint_commands_,"processed_joint_commands");
    memory_->getOrAddBlockByName(sim_effectors_,"sim_effectors");

    core = new MotionCore(CORE_SIM, true, *memory_);
    fParsedVision = false;
    particleFilter = new PFLocalization( worldModel, bodyModel, core);

    parser = new Parser(worldModel, bodyModel, teamName, particleFilter,
                        vision_frame_info_,
                        frame_info_,
                        raw_joint_angles_,
                        raw_sensors_ );



    initBeamed = false;
    initialized = false;
    beamTime = -1;
    hoverTime = 2.25;

    fallState = 0;
    fallenLeft = false;
    fallenRight = false;
    fallenDown = false;
    fallenUp = false;
    fallTimeStamp = -1;
    fallTimeWait = -1;

    lastGetupRecoveryTime = -1.0;

    monMsg = "";

    // TODO: Treat paths more correctly? (system independent way)
    try {
        readSkillsFromFile( "./skills/stand.skl" );
        readSkillsFromFile( "./skills/kick.skl" );
        readSkillsFromFile( "./skills/kick_ik_0.skl" );
        readSkillsFromFile( "./skills/dajiao.skl");
        readSkillsFromFile( "./skills/left_block.skl");
        readSkillsFromFile( "./skills/longpass.skl");
        readSkillsFromFile( "./skills/middle_block.skl");
        readSkillsFromFile( "./skills/right_block.skl");
        readSkillsFromFile( "./skills/slowpass.skl");
    }
    catch( std::string& what ) {
        cerr << "Exception caught: " << what << endl;
        exit(1);
    }
    catch (std::exception& e)
    {
        cerr << e.what() << endl;
        exit(1);
    }

    // initialize just so reset Skill doesnt segfault
    skill = SKILL_STAND;
    resetSkills();
    resetKickState();

    // Uncomment this to use ground truth data for localization
    //worldModel->setUseGroundTruthDataForLocalization(true);
}

NaoBehavior::~NaoBehavior() {

    delete parser;
    delete worldModel;
    delete bodyModel;
    delete particleFilter;
    delete core;
}

string NaoBehavior::Init() {
    cout << "Loading rsg: " << "(scene " << rsg << ")" << endl;
    return "(scene " + rsg + ")";
}





string NaoBehavior::Think(const std::string& message) {

    //  cout << "(NaoBehavior) received message " << message << endl;

    fParsedVision = false;
    bool parseSuccess = parser->parse(message, fParsedVision);
    if(!parseSuccess && (worldModel->getPlayMode() != PM_BEFORE_KICK_OFF)) {
//    cout << "****************************************\n";
//    cout << "Could not parse message: " << message << "\n";
//    cout << "****************************************\n";
    }

    //  cout << "\nparseSuccess: " << parseSuccess << "\n";
    //  worldModel->display();
    bodyModel->refresh();
    if(fParsedVision) {
        if (!worldModel->isFallen()) {
            parser->processVision();
        } else {
            parser->processSightings(true /*fIgnoreVision*/);
        }
    }
    this->updateFitness();
    //  bodyModel->display();
    //  bodyModel->displayDerived();

    // Example usage of the roboviz drawing system and RVSender in rvdraw.cc.
    // Draw agent positions and orientations
    /*
    worldModel->getRVSender()->clearStaticDrawings();
    VecPosition pos = worldModel->getMyPosition();
    VecPosition dir = VecPosition(1,0,0);
    dir = dir.rotateAboutZ(-worldModel->getMyAngDeg());
    worldModel->getRVSender()->drawPoint(pos.getX(), pos.getY(), 10);
    worldModel->getRVSender()->drawLine(pos.getX(), pos.getY(), pos.getX()+dir.getX(), pos.getY()+dir.getY());
    */

    calculateAngles();


    if (frame_info_->start_time == -1) {
        frame_info_->start_time = frame_info_->seconds_since_start;
        vision_frame_info_->start_time = frame_info_->start_time;
    }
    frame_info_->seconds_since_start= frame_info_->seconds_since_start - frame_info_->start_time;

    raw_joint_angles_->values_[RHipYawPitch] = raw_joint_angles_->values_[LHipYawPitch];

    preProcessJoints();  // Apply the correct sign to the joint angles

    postProcessJoints(); // Flip the joint angles back

    string action;

    if (!mInit) {

        mInit = true;
        stringstream ss;
        ss << "(init (unum " << agentUNum << ")(teamname " << agentTeamName << "))";
        action = ss.str();
        return action;
    }

    if (worldModel->getLastPlayMode() != worldModel->getPlayMode() &&
            (worldModel->getPlayMode() == PM_BEFORE_KICK_OFF ||
             worldModel->getPlayMode() == PM_GOAL_LEFT ||
             worldModel->getPlayMode() == PM_GOAL_RIGHT)) {
        initBeamed = false;
    }

    // Record game score
    if (worldModel->getScoreLeft() != -1 && worldModel->getScoreRight() != -1) {
        scoreMe = worldModel->getSide() == SIDE_LEFT ? worldModel->getScoreLeft() : worldModel->getScoreRight();
        scoreOpp = worldModel->getSide() == SIDE_LEFT ? worldModel->getScoreRight() : worldModel->getScoreLeft();
    }


    if ((worldModel->getPlayMode() == PM_GOAL_LEFT || worldModel->getPlayMode() == PM_GOAL_RIGHT || worldModel->getPlayMode() == PM_BEFORE_KICK_OFF) && worldModel->getLastPlayMode() != worldModel->getPlayMode()) {
        beamTime = worldModel->getTime() + hoverTime;
    }

    else if(beamTime >= 0 && worldModel->getTime() >= beamTime) {
        //initialized = false;
        initBeamed = false;
        beamTime = -1.0;
    }


    if (worldModel->getPlayMode() != worldModel->getLastPlayMode()) {
        worldModel->setLastDifferentPlayMode(worldModel->getLastPlayMode());
    }
    worldModel->setLastPlayMode(worldModel->getPlayMode());

    if(!initialized) {
        if(!worldModel->getUNumSet() || !worldModel->getSideSet()) {
            //      cout << "UNum and side not received yet.\n";
            action = "";
            return action;
        }

        if(!initBeamed) {
            initBeamed = true;


            double beamX, beamY, beamAngle;

            // Call a virtual function
            // It could either be implemented here (real game)
            // or in the inherited classes
            // Parameters are being filled in the beam function.
            this->beam( beamX, beamY, beamAngle );
            stringstream ss;
            ss << "(beam " << beamX << " " << beamY << " " << beamAngle << ")";
            particleFilter->setForBeam(beamX, beamY, beamAngle);
            action = ss.str();
            return action;
        }
        else {
            // Not Initialized
            bodyModel->setInitialHead();
            bodyModel->setInitialArm(ARM_LEFT);
            bodyModel->setInitialArm(ARM_RIGHT);
            bodyModel->setInitialLeg(LEG_LEFT);
            bodyModel->setInitialLeg(LEG_RIGHT);
            initialized = true;
        }
    }

    if(!initBeamed) {
        initBeamed = true;

        double beamX, beamY, beamAngle;

        // Call a virtual function
        // It could either be implemented here (real game)
        // or in the inherited classes - for optimization agent.
        // Parameters are being filled in the beam function.
        this->beam( beamX, beamY, beamAngle );
        stringstream ss;
        ss << "(beam " << beamX << " " << beamY << " " << beamAngle << ")";
        particleFilter->setForBeam(beamX, beamY, beamAngle);
        action = ss.str();
    }

    frame_info_->frame_id++;
    act();

    worldModel->getRVSender()->refresh();

    action = action + composeAction();

    //std::cout << "Sending action: " << action << "\n";
    return action;
}

int NaoBehavior::whogo()
{
     int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = 6; i < 9; ++i)
    {
        VecPosition temp;
        int playerNum = i ;
        if (worldModel->getUNum() == playerNum) 
	{
            // This is us
           
            temp = worldModel->getMyPosition();
            
        } 
        else 
	{
            WorldObject* teammate = worldModel->getWorldObject( i );
           if (teammate->validPosition)
	   {
                temp = teammate->pos;
           }
            else 
	    {
                continue;
            }
        }
        temp.setZ(0);


        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall)//&&(!checkingFall())) 
	{
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }
    if(Comparison())
    return playerClosestToBall;
    else
    return 100;//返回一个非补位球员的编号
    
}

//得到我方离球最近球员的距离
double NaoBehavior::woplayjuli(VecPosition target = {INF,INF,INF})
{

    if(target == VecPosition(INF,INF,INF))
        target = ball;

    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i)
    {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) 
	{
            // This is us
           
            temp = worldModel->getMyPosition();
            
        } 
        else 
	{
            WorldObject* teammate = worldModel->getWorldObject( i );
           if (teammate->validPosition)
	   {
                if(!worldModel->getFallenTeammate(i))
                    temp = teammate->pos;
                else{
                    continue;
                }
           }
            else 
	    {
                continue;
            }
        }
        temp.setZ(0);


        double distanceToBall = temp.getDistanceTo(target);
        if (distanceToBall < closestDistanceToBall)//&&(!checkingFall())) 
	{
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }
    return closestDistanceToBall;
}


//得到敌方离球最近的球员的距离

double  NaoBehavior::opplayjuli(VecPosition target = {INF,INF,INF})
{

    if(target == VecPosition(INF,INF,INF))
        target = ball;

    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_OPPONENT1; i < WO_OPPONENT1+NUM_AGENTS; ++i)
    {
   
      WorldObject* opponent = worldModel->getWorldObject( i );
           if (!opponent->validPosition)
	   {            
                continue;
       }
        VecPosition opp=opponent->pos;
        opp.setZ(0);

        double distanceToBall = opp.getDistanceTo(target);
        if (distanceToBall < closestDistanceToBall) 
	    {
            playerClosestToBall = i;
            closestDistanceToBall = distanceToBall;
        }
    }
    
    return closestDistanceToBall;
}

bool NaoBehavior:: Comparison(){
    if(woplayjuli()>=opplayjuli())
    return true;
    else 
    return false;
}

void NaoBehavior::act() {
    refresh();

    const double LAST_LINE_SIGHTING_THRESH = 0.1;
    if (worldModel->getTime()-worldModel->getLastLineSightingTime() > LAST_LINE_SIGHTING_THRESH) {
        worldModel->setLocalized(false);
    }


    // If the ball gets too far away, reset kick state
    if(me.getDistanceTo(ball) > 1) {
        resetKickState();
    }

    //worldModel->getRVSender()->drawPoint("me", me.getX(), me.getY(), 20);
    int pm = worldModel->getPlayMode();
    bool resetForKickoff = pm == PM_BEFORE_KICK_OFF || pm == PM_GOAL_LEFT || pm == PM_GOAL_RIGHT;



    if(checkingFall()) {
        resetSkills();
        bodyModel->setUseOmniWalk(false);
        return;
    }
    else if(resetForKickoff) {
        if (beamablePlayMode() && (worldModel->isFallen() || worldModel->getTime() <= beamTime)) {
            initBeamed = false;
        }
        resetSkills();
        skill = SKILL_STAND;
        core->move(0,0,0);
        velocity.paramSet = WalkRequestBlock::PARAMS_DEFAULT;
    }
    else {
        if(skills[skill]->done( bodyModel, worldModel) ||
                bodyModel->useOmniWalk()) {
            skills[skill]->reset();
            resetScales();
            SkillType currentSkill = selectSkill();


            if (currentSkill != SKILL_WALK_OMNI) {
                velocity.paramSet = WalkRequestBlock::PARAMS_DEFAULT;
            }

            bodyModel->setUseOmniWalk(true);
            switch(currentSkill) {
            case SKILL_WALK_OMNI:
                core->move(velocity.paramSet, velocity.x, velocity.y, velocity.rot);
                break;
            case SKILL_STAND:
                core->move(0,0,0);
                break;
            default:
                bodyModel->setUseOmniWalk(false);
            }

            if (bodyModel->useOmniWalk()) {
                resetSkills();
            } else {

                /*EnumParser<SkillType> enumParser;
                cout << "Skill: " << enumParser.getStringFromEnum(skill) << endl;*/

                // Transitions, coding a finite state machine...

                SkillType lastSkill = worldModel->getLastSkill();
                skill = currentSkill;


            }
        }
    }
    //  cout << "Executing: " << EnumParser<SkillType>::getStringFromEnum(skill) << endl;
    //  cerr << "Selected skill: " << SkillType2Str[skill] << " time: " << worldModel->getTime() << endl;
    //    LOG_ST(skill);
    skills[skill]->execute( bodyModel, worldModel );



    worldModel->setLastSkill(skill);
    // to be used by odometry
    if (bodyModel->useOmniWalk()) {
        worldModel->addExecutedSkill(SKILL_WALK_OMNI);
    } else {
        worldModel->addExecutedSkill( skill );
    }




    //Set the head turn behavior
    VecPosition me = worldModel->getMyPosition();
    me.setZ(0);
    ball = worldModel->getBall();
    ball.setZ(0);
    // Currently, every 2 seconds
    static double panOffset = drand48() * 4.0;
    int panState = ( static_cast<int>( worldModel->getTime()+panOffset ) ) % 4;
    double ballDistance, ballAngle;

    getTargetDistanceAndAngle(ball, ballDistance, ballAngle);
    //SkillType lastSkill = worldModel->getLastSkill();

    if (worldModel->isFallen()) {
        bodyModel->setScale(EFF_H1, 0.5);
        bodyModel->setTargetAngle(EFF_H1, 0);
    } else if (ballDistance < 1.0 && worldModel->getWorldObject(WO_BALL)->validPosition) {
        // close to the ball, focusing on the ball and turning head 30 degrees
        if( panState == 0 || panState == 2 ) {
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, ballAngle);
        } else {
            int direction = (panState == 1) ? 1 : -1;
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, ballAngle+(direction*30.0));
        }
    } else {
        // default behavior
        if( panState == 0 || panState == 2 ) {
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, 0);
        } else {
            int direction = (panState == 1) ? 1 : -1;
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, direction*120);// 30.0); // 120.0);
        }
    }
}


/*
 * Throws string
 */
void NaoBehavior::readSkillsFromFile(const std::string &filename)
{
    //  cerr << "Loading skills from file " << filename << endl;

    // Load a skill file to memory. Assuming a file is < 4K

    istream *skillFile;
    ifstream infile;
    stringstream instring;

    //关闭后读取文件参数,开启后读取字符串参数(打包在可执行文件内)
    if(read_paramfiles){
        instring =stringstream(Params::getInstance().getKickParams(filename));
        skillFile = &(instring);
    }
    else{
        infile = ifstream(filename,ios::in);
        skillFile = &(infile);
    }

    int buffsize = 65536;
    char buff[buffsize];
    int numRead;

    //fstream skillFile(filename.c_str(), ios_base::in);
    //stringstream skillFile(Params::getInstance().getKickParams(filename));

    skillFile->read(buff, buffsize);
    if (!skillFile->eof())
    {
        throw "failed to read the whole skill file " + filename;
    }
    numRead = skillFile->gcount();

    // padding with \0 at the end
    buff[numRead] = '\0';

    // Preprocessing: replace parameters by values.

    string skillDescription("");
    skillDescription.reserve(buffsize);
    for (int i = 0; i < numRead; ++i)
    {
        char c = buff[i];
        if (c == '$')
        {
            // parameter - replace it

            string param("");
            i += 1;
            while (i < numRead && (isalnum(buff[i]) || buff[i] == '_'))
            {
                param += buff[i];
                ++i;
            }

            map<string, string>::const_iterator it = namedParams.find(param);
            if (it == namedParams.end())
            {
                throw "Missing parameter in skill file " + filename + ": " + param;
            }
            skillDescription += it->second;

            if (i < numRead)
                skillDescription += buff[i];
        }
        else
        {
            // not a param, just concatenate c
            skillDescription += c;
        }
    }

    // Parse

    SkillParser parser(skills, bodyModel);
    parse_info<iterator_t> info = parse(skillDescription.c_str(),
                                        parser,
                                        (space_p | comment_p("#")));

    // check results
    if (info.hit)
    {
        //    cout << "-------------------------\n";
        //    cout << "Parsing succeeded\n";
        //    cout << "-------------------------\n";
        //    cout << "stop "  << info.stop << endl;
        //    cout << "full " << info.full << endl;
        //    cout << "length " << info.length << endl;
    }
    else
    {
        cout << "-------------------------\n";
        cout << "Parsing failed\n";
        //            cout << "stopped at: \": " << info.stop << "\"\n";
        cout << "-------------------------\n";
        //    throw "Parsing failed";
    }
}


bool NaoBehavior::isRightSkill( SkillType skill ) {
    string skillStr = EnumParser<SkillType>::getStringFromEnum( skill );
    return skillStr.find("RIGHT") != string::npos;
}

bool NaoBehavior::isLeftSkill( SkillType skill ) {
    string skillStr = EnumParser<SkillType>::getStringFromEnum( skill );
    return skillStr.find("LEFT") != string::npos;
}


double NaoBehavior::
trim(const double& value, const double& min, const double&max)
{
    double ret;
    if (value > max)
        ret = max;
    else if (value < min)
        ret = min;
    else
        ret = value;

    return ret;
}


 double NaoBehavior:: GetRand()
    {
    	srand(time(NULL));
     
    	int iRand = rand();
     
    	iRand %= 10000; // 取0 ~ 9999

        iRand=iRand-5000;
        if(ball.getX()>11.9)
        return 0;
        else
        {
        if(ball.getY()<=0)
    	return ((abs(iRand / 10000.0))-1.25); // 注意加".0" iRand的值在-0.5～0.49
        else
        return ((abs(iRand / 10000.0))+0.75);
        }
    	// PS：保留几位小数就合适求舍，求商
    }

 double NaoBehavior:: GetFand()
 {

   srand(time(NULL));
     
    	int iRand = rand();
     
    	iRand %= 10000; // 取0 ~ 9999

        iRand=iRand-5000;
     
    	return   ((abs(iRand / 10000.0))-0.25); // 注意加".0" iRand的值在-0.5～0.49
 }


void NaoBehavior::calculateAngles() {

    float  accX = raw_sensors_->values_[accelX];
    float  accY = raw_sensors_->values_[accelY];
    float  accZ = raw_sensors_->values_[accelZ];

    raw_sensors_->values_[angleX] = atan2(accY,accZ);
    raw_sensors_->values_[angleY] = -atan2(accX,accZ);

    //raw_sensors_->values_[gyroX] = 0; // = 1000000.0;
    //raw_sensors_->values_[gyroY] = 0; //= 1000000.0;
}

void NaoBehavior::preProcessJoints() {
    for (int i=0; i<NUM_JOINTS; i++) {
        processed_joint_angles_->values_[i] = spark_joint_signs[i] * raw_joint_angles_->values_[i];
    }
}

void NaoBehavior::postProcessJoints() {
    raw_joint_commands_->angle_time_ = processed_joint_commands_->angle_time_;
    raw_joint_commands_->stiffness_time_ = processed_joint_commands_->stiffness_time_;
    for (int i=0; i<NUM_JOINTS; i++) {
        raw_joint_commands_->angles_[i] = spark_joint_signs[i] * processed_joint_commands_->angles_[i]; // apply joint signs to convert to the robot's reference frame
        raw_joint_commands_->stiffness_[i] = processed_joint_commands_->stiffness_[i];
    }
    raw_joint_commands_->send_stiffness_ = processed_joint_commands_->send_stiffness_;
    processed_joint_commands_->send_stiffness_ = false;
}

void NaoBehavior::resetSkills() {
    skills[skill]->reset();

    skill = SKILL_STAND;
    skillState = 0;

    kickDirection = VecPosition(1.0, 0, 0);

    resetScales();

    kickType = KICK_FORWARD;

    skills[worldModel->getLastSkill()]->reset();
    worldModel->setLastSkill(skill);

    bodyModel->setUseOmniWalk(true);
}

void NaoBehavior::resetScales() {
    for (int e = int(EFF_H1); e < int(EFF_NUM); e++) {
        bodyModel->setScale(e, 1.0);
    }
}


// Determines whether a collision will occur while moving to a target, adjusting accordingly when necessary
VecPosition NaoBehavior::collisionAvoidance(bool avoidTeammate, bool avoidOpponent, bool avoidBall, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, bool fKeepDistance) {
    // Obstacle avoidance
    VecPosition closestObjPos = VecPosition(100, 100, 0);
    double closestObjDistance = me.getDistanceTo(closestObjPos);

    // Avoid the ball if flag is set
    if(avoidBall) {
        if (avoidball()) {
            closestObjPos = ball;
            closestObjDistance = me.getDistanceTo(ball);
        }
    }

    // Avoid all of your teamates if flag is set
    if(avoidTeammate) {
        for(int i = WO_TEAMMATE1; i <= WO_TEAMMATE11; ++i) {
            // Skip ourself
            if (worldModel->getUNum() == i - WO_TEAMMATE1 + 1) {
                continue;
            }
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition == true) {
                VecPosition temp = teammate->pos;
                temp.setZ(0);
                if (abs(me.getAngleBetweenPoints(target, temp)) < 90.0) {
                    if (!fKeepDistance && me.getDistanceTo(temp) > me.getDistanceTo(target)) {
                        continue;
                    }
                    double distance = me.getDistanceTo(temp);
                    if (distance < closestObjDistance) {
                        closestObjDistance = distance;
                        closestObjPos = temp;
                    }
                }
            }
        }
    }

    // Avoid opponents if flag is set
    if(avoidOpponent) {
        if (closestObjDistance > PROXIMITY_THRESH) {
            for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
                WorldObject* opponent = worldModel->getWorldObject( i );
                if (opponent->validPosition == true) {
                    VecPosition temp = opponent->pos;
                    temp.setZ(0);
                    if (abs(me.getAngleBetweenPoints(target, temp)) < 90.0 &&
                            me.getDistanceTo(temp) < me.getDistanceTo(target)) {
                        double distance = me.getDistanceTo(temp);
                        if (distance < closestObjDistance) {
                            closestObjDistance = distance;
                            closestObjPos = temp;
                        }
                    }
                }
            }
        }
    }

    // Determine where you need to move to avoid the closest object you want to avoid
    if (closestObjDistance <= PROXIMITY_THRESH) {
        VecPosition originalTarget = target;
        target = collisionAvoidanceCorrection(me, PROXIMITY_THRESH, COLLISION_THRESH, target, closestObjPos);
    }

    return target;
}

VecPosition NaoBehavior::collisionAvoidanceCorrection(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle) {
    double obstacleDist = start.getDistanceTo(obstacle);

    if (abs(start.getAngleBetweenPoints(target, obstacle)) >= 90.0 ||
            obstacleDist > PROXIMITY_THRESH) {
        return target;
    }


    VecPosition obstacleDir = (obstacle-start).normalize();

    VecPosition left90 = start + VecPosition(0, 0, 1).crossProduct(obstacleDir)*1.0;
    VecPosition right90 = start - VecPosition(0, 0, 1).crossProduct(obstacleDir)*1.0;
    if (target.getDistanceTo(left90) > target.getDistanceTo(right90)) {
        target = right90;
    } else {
        target = left90;
    }

    if (obstacleDist <= COLLISION_THRESH) {
        // We're way too close so also back away
        target += (start-obstacle).normalize()*1.0;
    }
    return target;
}

VecPosition NaoBehavior::collisionAvoidanceApproach(double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle) {
    return collisionAvoidanceApproach(me, PROXIMITY_THRESH, COLLISION_THRESH, target, obstacle);
}

VecPosition NaoBehavior::collisionAvoidanceApproach(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle) {
    double distanceToObstacle = start.getDistanceTo(obstacle);
    if (fabs(start.getAngleBetweenPoints(target, obstacle)) >= 90.0 ||
            distanceToObstacle > start.getDistanceTo(target)) {
        return target;
    }

    if (distanceToObstacle <= PROXIMITY_THRESH) {
        return collisionAvoidanceCorrection(start, PROXIMITY_THRESH, COLLISION_THRESH, target, obstacle);
    }

    VecPosition start2Target = target-start;
    VecPosition start2TargetDir = VecPosition(start2Target).normalize();
    VecPosition start2Obstacle = obstacle-start;
    VecPosition start2ObstacleDir = VecPosition(start2Obstacle).normalize();


    VecPosition closestPathPoint = start+
                                   (start2TargetDir*(start2Obstacle.dotProduct(start2TargetDir)));

    double pathDistanceFromObstacle = (obstacle-closestPathPoint).getMagnitude();
    VecPosition originalTarget = target;
    if (pathDistanceFromObstacle < PROXIMITY_THRESH) {
        target = obstacle + (closestPathPoint-obstacle).normalize()*PROXIMITY_THRESH;
    }
    return target;

}

//******************************************************************************************************************************
//写出守门员角平分线站位
SkillType NaoBehavior::angle_bisecton()
{
   //首先我们得到两个门柱和球的坐标
   VecPosition left_goalpost=VecPosition(-14.5,1,0);
   VecPosition right_goalpost=VecPosition(-14.5,-1,0);
   VecPosition ball=worldModel->getBall();
   //由于守门员的X值是确定的所以在这里只需要求出Y值，并且Y的取值范围应该在-1和1之间
   double y;
   //求出三角形中两条直线的方程
   double k1;
   double b1;
   k1=(ball.getY()-left_goalpost.getY())/(ball.getX()-left_goalpost.getX());
   b1=left_goalpost.getY()-k1*left_goalpost.getX();
   double k2;
   double b2;
   k2=(ball.getY()-right_goalpost.getY())/(ball.getX()-right_goalpost.getX());
   b2=right_goalpost.getY()-k2*right_goalpost.getX();
   for(y=-1;y<=1;y=y+0.000001)
   {
       if(abs(y-k1*left_goalpost.getX()-b1)/sqrt(1+k1*k1)==abs(y-k2*right_goalpost.getX()-b2)/sqrt(1+k2*k2))
       break;
   }
   VecPosition target=VecPosition(-14.5,y,0);
   VecPosition center=ball;
       VecPosition localCenter = worldModel->g2l(center);
       SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
       target = collisionAvoidance(true /*teammate*/, true/*opponent*/, false/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
       if(me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
            return SKILL_WALK_OMNI;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }

}
//写出守门员的策略
SkillType NaoBehavior::goalkeeper()
{
    VecPosition ball=worldModel->getBall();
    if(ball.getX()>-6)
    return angle_bisecton();
    else 
    return defend();
}
//写出守门员扑球
//写出后卫和守门员的双重防守，即圆柱型防守
SkillType NaoBehavior::cylinder()
{
    int playerClosestToBall=belong();
    double  sjs=GetRand();
    double  jsj=GetFand();
    if (playerClosestToBall == worldModel->getUNum()) 
    return kickBall(KICK_DRIBBLE, VecPosition(15, sjs, 0));
    double x;
    double y;
    double k;
    double b;
    VecPosition ball=worldModel->getBall();
    VecPosition cen=VecPosition(-15,0,0);
    k=(cen.getY()-ball.getY())/(cen.getX()-ball.getX());
    b=cen.getY()-k*cen.getX();
    for(y=-4;y<=4;y=y+0.000001)
    for(x=-15;x<=-11;x=x+0.000001)
    {
        if((x+15)*(x+15)+y*y==16&&k*x+b==y)
        break;
    }
    VecPosition target=VecPosition(x,y,0);
    VecPosition center=ball;
       VecPosition localCenter = worldModel->g2l(center);
       SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
       target = collisionAvoidance(true /*teammate*/, true/*opponent*/, false/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
       if(me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
            return SKILL_WALK_OMNI;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
}

SkillType NaoBehavior::mark(int m)
{
    int num=0;
    VecPosition op[11];
    VecPosition target;
    if(m==whogo())
    {
         target=ball;
    }
    else
    {
        for(int i = WO_OPPONENT1; i < WO_OPPONENT1+NUM_AGENTS; ++i)
    {
   
      WorldObject* opponent = worldModel->getWorldObject( i );
           if (!opponent->validPosition)
	   {            
                continue;
       }
        VecPosition opp=opponent->pos;
        opp.setZ(0);
        op[num++]=opp;
    }
    VecPosition ball=worldModel->getBall();
    double k=(op[m].getY()-ball.getY())/(op[m].getX()-ball.getX());
    double b=ball.getY()-k*ball.getX();
    if(op[m].getX()>ball.getX())
    {
       target=VecPosition(op[m].getX()-0.5,k*(op[m].getX()-0.2)+b,0);
    }
    else if(op[m].getX()==ball.getX())
    {
        target=VecPosition(op[m].getX(),k*(op[m].getX())+b,0);
    }
    else
    {
        target=VecPosition(op[m].getX()+0.5,k*(op[m].getX()+0.2)+b,0);
    }
    }
    if(target.getX()>0)
    {
        return attackposit1(m);
    }
    else
    {
       VecPosition center=ball;
       VecPosition localCenter = worldModel->g2l(center);
       SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
       target = collisionAvoidance(true /*teammate*/, true/*opponent*/, false/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
       if(me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
            return SKILL_WALK_OMNI;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
    } 
}



SkillType NaoBehavior::defend()
{
    int playerClosestToBall=belong();
    double  sjs=GetRand();
    double  jsj=GetFand();
    switch(worldModel->getPlayMode())
     {
        case PM_KICK_OFF_LEFT:
        switch(worldModel->getSide())
        {
          case 0:
          {
           if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_DAJIAO, VecPosition(3,-1,0)); 
          }
          case 1:
          return SKILL_STAND;
        }
        case PM_KICK_OFF_RIGHT:
        switch(worldModel->getSide())
        {
          case 1:
          {
           if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_DAJIAO, VecPosition(3,-1,0)); 
           else
           return SKILL_STAND;
          }
          case 0:
          return SKILL_STAND;
        }
        case PM_PLAY_ON:
         {
         if (playerClosestToBall == worldModel->getUNum())
         { 
         return avoid();
         }
         else
         break;
         }
        case PM_KICK_IN_LEFT:
        switch(worldModel->getSide())
        {
          case 0:
          {
          if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_DAJIAO, VecPosition(14,0,0));

           else
           return SKILL_STAND;
          }
          case 1:
          return  SKILL_STAND;
        }
        case PM_KICK_IN_RIGHT:
        switch(worldModel->getSide())
        {
          case 1:
           {
           if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_DAJIAO, VecPosition(14,0,0));
           else
           return SKILL_STAND;
          }
          case 0:
          return  SKILL_STAND; 
        }
        case PM_CORNER_KICK_LEFT:
        switch(worldModel->getSide())
        {
          case 0:
          {
          if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_FORWARD, VecPosition(14,0,0));
           else
           return SKILL_STAND;
          }
          case 1:
          return  SKILL_STAND; 
        }
        case PM_CORNER_KICK_RIGHT:
        switch(worldModel->getSide())
        {
          case 1:
          {
          if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_FORWARD, VecPosition(14,0,0));
           else
           return SKILL_STAND;
          }
          case 0:
          return  SKILL_STAND; 
        }
        case PM_GOAL_KICK_LEFT:
         switch(worldModel->getSide())
        {
          case 0:
          {
           if (1 == worldModel->getUNum())
           return kickBall(KICK_DAJIAO, VecPosition(14,-0.1,0)); 
           else
           return SKILL_STAND;
          }
          case 1:
          return  SKILL_STAND;           //对方在对方球门前开球
        }
        case PM_GOAL_KICK_RIGHT:
         switch(worldModel->getSide())
        {
          case 1:
          {
           if (1 == worldModel->getUNum())
           return kickBall(KICK_DAJIAO, VecPosition(14,-0.1,0)); 
           else
           return SKILL_STAND;
          }
          case 0:
          return  SKILL_STAND;            //对方在对方球门前开球
        }
        case PM_OFFSIDE_LEFT:
         switch(worldModel->getSide())
        {
          case 0:
          {
           if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_FORWARD, VecPosition(14,-0.1,0)); 
           else
           return SKILL_STAND;
          }
          case 1:
          return SKILL_STAND;          
        }
        case PM_OFFSIDE_RIGHT:
         switch(worldModel->getSide())
        {
          case 1:
          {
           if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_FORWARD, VecPosition(14,-0.1,0)); 
           else
           return  SKILL_STAND;
          }
          case 0:
          return SKILL_STAND;           
        }
        case PM_FREE_KICK_LEFT:
        switch(worldModel->getSide())
        {
          case 0:
          {
          if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_FORWARD, VecPosition(14,0,0));

           else
           return SKILL_STAND;
          }
          case 1:
          return  SKILL_STAND; 
        }
        case PM_FREE_KICK_RIGHT:
        switch(worldModel->getSide())
        {
          case 1:
           {
          if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_FORWARD, VecPosition(14,0,0));
           else
           return SKILL_STAND;
          }
          case 0:
          return  SKILL_STAND; 
        }

     }
      switch(worldModel->getUNum())
  {
    
      case 3:
      return Anticollision4(-5.5,-5);
      case 2:
      return Anticollision2(-ball.getX()-12,-ball.getY(),-12,-9);
      case 1:
      return Anticollision2(-ball.getX()-14,-ball.getY(),-15,-12);
    
  }
}

bool isInsideEllipse(double x, double y, double center_x, double center_y, double a, double b) {
    double value = pow((x - center_x) / a, 2) + pow((y - center_y) / b, 2);
    if (value <= 1) {
        return true;
    }
    return false;
}

double getLineY(double x,double ax,double ay,double bx,double by){
    return ((by - ay) / (bx - ax)) * (x - ax) + ay; 
}

VecPosition getPerpendicularFoot(const VecPosition& a, const VecPosition& l1, const VecPosition& l2) {
    double x1 = l1.getX(), y1 = l1.getY();
    double x2 = l2.getX(), y2 = l2.getY();
    double x0 = a.getX(), y0 = a.getY();
    double k = (y2 - y1) / (x2 - x1);
    double b = y1 - k * x1;
    double x = (k * y0 + x0 - k * b) / (k * k + 1);
    double y = k * x + b;
    if ((x >= x1 && x <= x2) || (x >= x2 && x <= x1)) {
        return VecPosition(x,y);
    } else {
        double dist1 = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
        double dist2 = sqrt(pow(x2 - x0, 2) + pow(y2 - y0, 2));
        if (dist1 < dist2) {
            return VecPosition(x1,y1);
        } else {
            return VecPosition(x2,y2);
        }
    }
}

bool inArea(VecPosition target){
    if(abs(target.getX())<15&&abs(target.getY())<10)
        return true;
    return false;
}

SkillType NaoBehavior::closeOtherGoal(){
    VecPosition ball = worldModel->getMyPosition();
    double x = ball.getX();
    double y = ball.getY();

    //if(isInsideEllipse(x,y,15,0,1,2.05));
    if(x>14)
    {
        VecPosition foot = getPerpendicularFoot({x,y},{15,0.9},{15,-0.9});
        return kickBall(KICK_DRIBBLE,foot);
    }
        

    return SKILL_NONE;
}

SkillType NaoBehavior::closeOtherCorner(){
    VecPosition ball = worldModel->getMyPosition();
    double x = ball.getX();
    double y = ball.getY();

    double maxY = getLineY(x,15.4,0,15.0,1.05);
    if(abs(y)>maxY)
    {
        return kickBall(KICK_IK,VecPosition(14,0));
    }
    
    if(ball.getDistanceTo(VecPosition(15,2.05))<1)
    {
        return kickBall(KICK_DRIBBLE,VecPosition(14.5,1.05));
    }
    else if(ball.getDistanceTo(VecPosition(15,-2.05))<1)
    {
        return kickBall(KICK_DRIBBLE,VecPosition(14.5,-1.05));
    }
    

    return SKILL_NONE;

}

SkillType NaoBehavior::kickik(vector<pair<VecPosition,float>>* scoreSheet){
    float maxScore = -INF;
    VecPosition bestPos = VecPosition(INF,INF,INF);

    for(int i=0;i<scoreSheet->size();i++){
        if(scoreSheet->at(i).second>maxScore){
            bestPos = scoreSheet->at(i).first;
            maxScore = scoreSheet->at(i).second;
        }
    }

    SkillType skill = SKILL_NONE;
    //cout<<bestPos<<endl;
    if(inArea(bestPos))
    {
        if(abs(bestPos.getDistanceTo(ball)-5)<abs(bestPos.getDistanceTo(ball)-11))
        {
            return kickBall(KICK_IK,bestPos);
        }
        else{
            return kickBall(KICK_DAJIAO,bestPos);
        }
    }

    return kickBall(KICK_IK,VecPosition(14,0));

    delete(scoreSheet);
}

void NaoBehavior::pass(){
    VecPosition ball = worldModel->getBall();
    //离球门过近不会申请pass(pass模式后10s内不能进球)
    if(ball.getDistanceTo(VecPosition(15.5,0))<6)
        return;

    double opponent_distance = opplayjuli();
    double teammate_distance = woplayjuli();
    if(opponent_distance<=1.25&&teammate_distance<opponent_distance+1.0&&teammate_distance>0.2)
    {
        if(worldModel->getSide()==SIDE_LEFT){
            PutMessage("(pass left)");
        }   
        else{
            PutMessage("(pass right)");
        }
            
        
    }
}

vector<pair<VecPosition,float>>* NaoBehavior::scorePointList(){  
    

  VecPosition ball = worldModel->getBall();
  VecPosition me = worldModel->getMyPosition();

  vector<pair<VecPosition,float>>* scoreSheet = new vector<pair<VecPosition,float>>();

  static vector<double> distances = vector<double>({5,12});

  for (double a = 0; a < 2 * M_PI; a+=M_PI/30)
  {
    for (int i = 0;i<distances.size();i++){
        double x = ball.getX() + distances[i]*cos(a);
        double y = ball.getY() + distances[i]*sin(a);

        if(!inArea({x,y,0}))
            continue;

        if(judgePlayerCollision(VecPosition(x,y)))
            continue;
        

        float score = 0.0;

        score += VecPosition(x,y).getDistanceTo(VecPosition(15,0,0)) * -0.05;
        score += opplayjuli(VecPosition(x,y)) * -0.5;
        score += woplayjuli(VecPosition(x,y)) * 0.3;

        scoreSheet->push_back(pair<VecPosition,float>({VecPosition(x,y),score}));
    }
  }

  
  
  return scoreSheet;
}
bool judgeCollision(const VecPosition &start, const VecPosition &target, const VecPosition &obstacles, double bot_obstacles_box_size = 0.6)
{
    // double bot_obstacles_box_size = 0.7;
    auto tar = target;
    if (tar.getDistanceTo(obstacles) < bot_obstacles_box_size)
        return true;

    double x1 = start.getX();
    double y1 = start.getY();
    double x2 = target.getX();
    double y2 = target.getY();
    double x0 = obstacles.getX();
    double y0 = obstacles.getY();

    if (x2 - x1 == 0)
        x2 += 1e-6;

    // double theta21=0;
    // if(y2-y1<=0)theta21+=M_PI;
    double theta21 = atan((y2 - y1) / (x2 - x1));

    // double theta20 = atan((y2 - y0) / (x2 - x0));
    double theta10 = atan((y1 - y0) / (x1 - x0));

    // if (theta20 - M_PI / 2.0 < theta10 && theta10 < theta20 + M_PI / 2.0)
    //     continue;

    double a2 = pow(x1 - x0, 2) + pow(y1 - y0, 2);
    double b2 = pow(x2 - x0, 2) + pow(y2 - y0, 2);
    double c2 = pow(x2 - x1, 2) + pow(y2 - y1, 2);

    if (c2 <= a2 + b2)
        return false;

    double delta_x = x1 - x2;
    double delta_y = y1 - y2;

    double A = -delta_y;
    double B = delta_x;
    double C = x2 * delta_y - y2 * delta_x;

    double distance2 = pow((A * x0 + B * y0 + C), 2) / (pow(A, 2) + pow(B, 2));

    if (distance2 <= pow(bot_obstacles_box_size, 2))
    {
        return true;
    }

    return false;
}

bool NaoBehavior::judgePlayerCollision(VecPosition target)
{
    for(int p = WO_TEAMMATE1;p<=WO_OPPONENT11;p++){
        WorldObject *ob = worldModel->getWorldObject(p);
        if(!ob->validPosition)
            continue;
        
        double ob_size = 0.5;
        if(ob->pos.getDistanceTo(ball)<1.5)
            ob_size = 0.1;
        if(judgeCollision(ball,target,ob->pos,ob_size))
            return true;

    }
    return false;
}

void pass(WorldModel* worldModel){
  int side = worldModel->getSide();
  if(side==SIDE_LEFT)
    PutMessage("(pass right)");
  else
    PutMessage("(pass left)");
}




SkillType NaoBehavior::stop()
{
    VecPosition goal= VecPosition(HALF_FIELD_X,-1,0);
    int playerClosestToBall;
    playerClosestToBall=belong();
    if(playerClosestToBall==worldModel->getUNum())
    return kickBall(KICK_IK,goal);
    else
    return posit4();
}


int NaoBehavior::belong()
{
    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) 
     {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) 
        {
            // This is us
            temp = worldModel->getMyPosition();
        } 
        else 
        {
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition) 
            {
                if(!worldModel->getFallenTeammate(i))
                    temp = teammate->pos;
                else
                    continue;
            } 
            else 
            {
                continue;
            }
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall) 
        {
                playerClosestToBall = playerNum;
                closestDistanceToBall = distanceToBall;
        }
     }
     return playerClosestToBall;
}

int NaoBehavior:: kic()
{
 if(ball.getX()>=13.9&&abs(ball.getY()<=0.49))
    return 0;
    else 
    return 1;
}

int NaoBehavior::kicc(){
    //0 代表球小于3米
    //1 代表球的距离在3到6米中间 
    //2 代表球的距离大于9米
    VecPosition target = VecPosition(15,0,0);
    if(me.getDistanceTo(target)<=3){
        return 0;
    }else if(me.getDistanceTo(target)<=9){
        return 1;
    }else{
        return 2;
    }

}

void NaoBehavior::setmsg(const string& msg){
    if (msg.empty())
    {
        return;
    }

    // prefix the message with it's payload length
    unsigned int len = htonl(msg.size());
    string prefix((const char*)&len,sizeof(unsigned int));
    string str = prefix + msg;
    if ( static_cast<ssize_t>(str.size()) != write( 3 ,str.data(), str.size())) {
        LOG_STR("could not put entire message: " + msg);
    }
}

//在这里写出先碰触到球还是先调整位置的函数
bool NaoBehavior:: avoidball(){
     if(worldModel->getPlayMode()==3){
          if(ball.getX()<=-14||(ball.getX()>=13.9&&ball.getY()>=0.49)||(ball.getX()>=13.9&&ball.getY()<=-0.49)||ball.getY()>=9||ball.getY()<=-9){
            return true;
          }else{
            return false;
          }
     }else{
          return true;
     }
     

} 




/*
//Play Modes
#define PM_BEFORE_KICK_OFF 0
#define PM_KICK_OFF_LEFT   1
#define PM_KICK_OFF_RIGHT  2
#define PM_PLAY_ON         3
#define PM_KICK_IN_LEFT    4
#define PM_KICK_IN_RIGHT   5
#define PM_GOAL_LEFT       6
#define PM_GOAL_RIGHT      7
#define PM_GAME_OVER       8

//Extra added.
#define PM_CORNER_KICK_LEFT       9     //角球
#define PM_CORNER_KICK_RIGHT      10
#define PM_GOAL_KICK_LEFT         11    //对方在对方球门前开球   门球
#define PM_GOAL_KICK_RIGHT        12    //对方在对方球门前开球
#define PM_OFFSIDE_LEFT           13     越位球
#define PM_OFFSIDE_RIGHT          14
#define PM_FREE_KICK_LEFT         15     任意球
#define PM_FREE_KICK_RIGHT        16
#define PM_DIRECT_FREE_KICK_LEFT  17     直接任意球左
#define PM_DIRECT_FREE_KICK_RIGHT 18

*/

/*//Play Modes
#define PM_BEFORE_KICK_OFF 0
#define PM_KICK_OFF_LEFT   1
#define PM_KICK_OFF_RIGHT  2
#define PM_PLAY_ON         3
#define PM_KICK_IN_LEFT    4
#define PM_KICK_IN_RIGHT   5
#define PM_GOAL_LEFT       6
#define PM_GOAL_RIGHT      7
#define PM_GAME_OVER       8

//Extra added.
#define PM_CORNER_KICK_LEFT       9
#define PM_CORNER_KICK_RIGHT      10
#define PM_GOAL_KICK_LEFT         11
#define PM_GOAL_KICK_RIGHT        12
#define PM_OFFSIDE_LEFT           13
#define PM_OFFSIDE_RIGHT          14
#define PM_FREE_KICK_LEFT         15
#define PM_FREE_KICK_RIGHT        16
#define PM_DIRECT_FREE_KICK_LEFT  17
#define PM_DIRECT_FREE_KICK_RIGHT 18
#define PM_PASS_LEFT              19
#define PM_PASS_RIGHT             20*/
//xin de ce lue
SkillType NaoBehavior::def()
{
    int playerClosestToBall=belong();
    double  sjs=GetRand();
    double  jsj=GetFand();
    switch(worldModel->getPlayMode())
    {
        case PM_KICK_OFF_LEFT:
        switch(worldModel->getSide())
        {
          case 0:
          {
           if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_DAJIAO, VecPosition(-3,1,0));
           else
           return SKILL_STAND; 
          }
          case 1:
          return SKILL_STAND;
        }
        case PM_KICK_OFF_RIGHT:
        switch(worldModel->getSide())
        {
          case 1:
          {
           if (playerClosestToBall == worldModel->getUNum())
           return kickBall(KICK_DAJIAO, VecPosition(-3,1,0));
           else
           return SKILL_STAND;
          }
          case 0:
          return SKILL_STAND;
        }
        case PM_PLAY_ON:
        {
         return attack();
        }
        case PM_KICK_IN_LEFT:
        switch(worldModel->getSide())
        {
          case 0:
          {
           if (playerClosestToBall == worldModel->getUNum())
           //return SKILL_STAND;
           return kickik(scorePointList());
           else
           return posit5();//gai
          }
          case 1:
          return posit1();//gai
        }
        case PM_KICK_IN_RIGHT:
        switch(worldModel->getSide())
        {
          case 1:
           {
           if (playerClosestToBall == worldModel->getUNum())
           return kickik(scorePointList());
           else
           return posit5();//gai
          }
          case 0:
          return posit1();//gai
        }
        case PM_CORNER_KICK_LEFT:
        switch(worldModel->getSide())
        {
          case 0:
          {
          if (playerClosestToBall == worldModel->getUNum())
          return kickik(scorePointList());
           else
           return posit6();
          }
          case 1:
          return posit1();
        }
        case PM_CORNER_KICK_RIGHT:
        switch(worldModel->getSide())
        {
          case 1:
          {
           if (playerClosestToBall == worldModel->getUNum())
           return kickik(scorePointList());
           else
           return posit6();
          }
          case 0:
          return posit1();
        }
        case PM_GOAL_KICK_LEFT:
         switch(worldModel->getSide())
        {
          case 0:
          {
           if (1 == worldModel->getUNum())
           return kickik(scorePointList());
           else
           return posit2();
          }
          case 1:
          return posit();           //对方在对方球门前开球
        }
        case PM_GOAL_KICK_RIGHT:
         switch(worldModel->getSide())
        {
          case 1:
          {
           if (1 == worldModel->getUNum())
           return kickik(scorePointList());
           else
           return posit2();
          }
          case 0:
          return posit();           //对方在对方球门前开球
        }
        case PM_OFFSIDE_LEFT:
         switch(worldModel->getSide())
        {
          case 0:
          {
           if (playerClosestToBall == worldModel->getUNum())
           return kickik(scorePointList());
           else
           return SKILL_WALK_OMNI;
          }
          case 1:
          return SKILL_WALK_OMNI;          
        }
        case PM_OFFSIDE_RIGHT:
         switch(worldModel->getSide())
        {
          case 1:
          {
           if (playerClosestToBall == worldModel->getUNum())
           return kickik(scorePointList());
           else
           return SKILL_WALK_OMNI;
          }
          case 0:
          return SKILL_WALK_OMNI;           
        }
        case PM_FREE_KICK_LEFT:
        switch(worldModel->getSide())
        {
          case 0:
          {
          if (playerClosestToBall == worldModel->getUNum())
           return kickik(scorePointList());
           else
           return posit5();
          }
          case 1:
          return posit1();
        }
        case PM_FREE_KICK_RIGHT:
        switch(worldModel->getSide())
        {
          case 1:
           {
           if (playerClosestToBall == worldModel->getUNum())
           return kickik(scorePointList());
           else
           return posit5();
          }
          case 0:
          return posit1();
        }
        case PM_PASS_LEFT:
        switch(worldModel->getSide())
        {
          case 0:
           {
           return attack();
          }
          case 1:
          return posit2();
        }
        case PM_PASS_RIGHT:
        switch(worldModel->getSide())
        {
          case 1:
           {
           return attack();
          }
          case 0:
          return posit2();
        }
    }
}

SkillType NaoBehavior::attackposit()
{
    double jsj=GetFand()*2;
    switch(worldModel->getUNum())
     {
       case 11:
       return Anticollision1(-1.5,2);
       case 10:
       return Anticollision1(-1.5,-2);
       case 9:
       return Anticollision4(0,0);
       case 8:
       if(ball.getX()<0)
       return Anticollision7(-1);
       else
       return goToTarget(VecPosition(13,-2,0));
       case 7:
       if(ball.getX()<0)
       return Anticollision7(1);
       else
       return goToTarget(VecPosition(13,2,0));
       case 6:
       return Anticollision5(-1.2,1.15,6);
       case 5:
       return Anticollision5(-1.5,0,5);
       case 4:
       return Anticollision5(-1.2,-1.25,4);
      }
}

SkillType NaoBehavior::defendposit()
{
    switch(worldModel->getUNum())
     {
      case 3:
      return Anticollision4(-5,3);
      case 2:
      //return Anticollision2(-ball.getX()-12,-ball.getY(),-12,-9);
      return Anticollision8(-11.8);
      case 1:
      //return Anticollision2(-ball.getX()-14,-ball.getY(),-15,-12);
      return Anticollision8(-14);
      }
}

SkillType NaoBehavior::attackposit1(int number)
{
    switch(number)
     {
       case 11:
       return Anticollision1(-1.5,2);
       case 10:
       return Anticollision1(-1.5,-2);
       case 9:
       return Anticollision4(0.5,1);
       case 8:
       return Anticollision1(1.2,1.2);
       case 7:
       return Anticollision1(1.2,-1.2);
       case 6:
       return Anticollision5(-1.2,1.15,6);
       case 5:
       return Anticollision5(-1.5,0,5);
       case 4:
       return Anticollision5(-1.2,-1.25,4);
      }
}
SkillType NaoBehavior::peopletopeople(){
    switch(worldModel->getUNum())
     {
       case 11:
       return mark(10);
       case 10:
       return mark(9);
       case 9:
       return mark(8);
       case 8:
       return mark(7);
       case 7:
       return mark(6);
       case 6:
       return mark(5);
       case 5:
       return mark(4);
       case 4:
       return mark(3);
     }
}

SkillType NaoBehavior::attack()
{
    double wo=woplayjuli();
    double op=opplayjuli();
    VecPosition ball=worldModel->getBall();
    VecPosition wogoal=VecPosition(-15,0,0);
    VecPosition opgoal=VecPosition(15,0,0);
    int playerClosestToBall=belong();
    double  sjs=GetRand();
    double  jsj=GetFand();
    double k=(wogoal.getY()-ball.getY())/(wogoal.getX()-ball.getX());
    double b=ball.getY()-k*ball.getX();
    VecPosition temp;
    switch(worldModel->getUNum()){
        case 11:
             if(playerClosestToBall==11)
             return avoid();
             else
             return attackposit();
        case 10:
            if(playerClosestToBall==10)
             return avoid();
             else
             return attackposit();
        case 9:
            if(playerClosestToBall==9)
             return avoid();
             else
             return attackposit();
        case 8:
            if(playerClosestToBall==8)
             return sideplayer();
             else
             return attackposit();
        case 7:
            if(playerClosestToBall==7)
             return sideplayer();
             else
             return attackposit();
        case 6:
            if(playerClosestToBall==6)
             return avoid();
             else
             return attackposit();
        case 5:
            if(playerClosestToBall==5)
             return avoid();
             else
             return attackposit();
        case 4:
            if(playerClosestToBall==4)
             return avoid();
             else
             return attackposit();
        case 3:
            if(playerClosestToBall==3)
             return avoid();
             else
             return defendposit();
        case 2:
            if(playerClosestToBall==2)
             return avoid();
             else
             return defendposit();
        case 1:
            if(playerClosestToBall==1)
             return avoid();
             else
             return defendposit();          
    }      
}

SkillType NaoBehavior::sideplayer(){
    double  sjs=GetRand();
    double  jsj=GetFand();
    VecPosition target = VecPosition(15,0,0);
    // 0 means ball is on op's Restricted area
    if(me.getDistanceTo(target)<10&&kic()==1){
                if(ball.getY()>0){
                    return kickBall(KICK_DAJIAO,VecPosition(15.0,-0.25,0));
                }else{
                    return kickBall(KICK_DAJIAO,VecPosition(15.0,0.25,0));
                }
    }else{
        return kickBall(KICK_DRIBBLE,VecPosition(14.5,sjs,0));
    }
    
}

SkillType NaoBehavior::avoid(){
    VecPosition closestObjPos = VecPosition(100, 100, 0);
    double closestObjDistance = me.getDistanceTo(closestObjPos);
    VecPosition target=VecPosition(15,0,0);
    double PROXIMITY_THRESH=0.30;
    //double COLLISION_THRESH=0.5;
    double  sjs=GetRand();
    double  jsj=GetFand();
    
    pass();

    if(worldModel->getPlayMode()==PM_PASS_LEFT)
    {
        //cout<<"pass"<<endl;
        if(worldModel->getSide()==SIDE_LEFT)
            return kickik(scorePointList());
    }
    else if(worldModel->getPlayMode()==PM_PASS_RIGHT){
        //cout<<"pass"<<endl;
        if(worldModel->getSide()==SIDE_RIGHT)
            return kickik(scorePointList());
    }


    SkillType skill = closeOtherCorner();
    if(skill!=SKILL_NONE)
    {
        //cout<<"closeCorner"<<endl;
        return skill;
    }


    skill = closeOtherGoal();
    if(skill!=SKILL_NONE)
    {
        //cout<<"closeGoal"<<endl;
        return skill;
    }
        

    


    if (closestObjDistance > PROXIMITY_THRESH) {
            for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
                WorldObject* opponent = worldModel->getWorldObject( i );
                if (opponent->validPosition == true) {
                    VecPosition temp = opponent->pos;
                    temp.setZ(0);
                    if (abs(me.getAngleBetweenPoints(target, temp)) < 90.0 &&
                            me.getDistanceTo(temp) < me.getDistanceTo(target)) {
                        double distance = me.getDistanceTo(temp);
                        if (distance < closestObjDistance) {
                            closestObjDistance = distance;
                            closestObjPos = temp;
                        }
                    }
                }
            }
    }


    if (closestObjDistance <= PROXIMITY_THRESH) 
    {
       double obstacleDist = me.getDistanceTo(closestObjPos);
       if (abs(me.getAngleBetweenPoints(target, closestObjPos)) >= 90.0 || obstacleDist > PROXIMITY_THRESH) {
            return kickBall(KICK_DRIBBLE,target);
       }else{
        VecPosition obstacleDir = (closestObjPos-me).normalize();
        VecPosition left90 = (me + VecPosition(0, 0, 1).crossProduct(obstacleDir)*1.0).normalize();
        VecPosition right90 = (me - VecPosition(0, 0, 1).crossProduct(obstacleDir)*1.0).normalize();
            if(ball.getX()>0){
                return kickBall(KICK_DRIBBLE,VecPosition(14.5,jsj,0));
            }else{
                if (target.getDistanceTo(left90) > target.getDistanceTo(right90)) {
                    return kickBall(KICK_DRIBBLE,right90);
                } else {
                    return kickBall(KICK_DRIBBLE,left90);
                }
            }
        }
    }else{
        //pass the ball
        if(ball.getX()<0){
           if(ball.getY()<0){
            return kickBall(KICK_DAJIAO,VecPosition(ball.getX()+7.0,-8, 0));
        }else{
            return kickBall(KICK_DAJIAO,VecPosition(ball.getX()+7.0,8, 0));
        }
        }else{
            if(kicc()==0){
                return kickBall(KICK_IK,VecPosition(15,0.25,0));
            }else{
                return kickBall(KICK_DRIBBLE,VecPosition(15,jsj,0));
            }
        }
    }
}

SkillType NaoBehavior::Anticollision1(double x,double y)
{
       
       VecPosition  target =VecPosition(ball.getX()+x,ball.getY()+y,0);
       VecPosition center=ball;
       VecPosition localCenter = worldModel->g2l(center);
       SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
       target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
        if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
              return SKILL_WALK_OMNI;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
}

SkillType NaoBehavior::Anticollision2(double x,double y,double z,double w)
{
        double wo=woplayjuli();
        double op=opplayjuli();
        VecPosition target = VecPosition(ball.getX()+x,ball.getY()+y,0);
        VecPosition center=ball;
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
        if(ball.getX()>z&&ball.getX()<=w&&ball.getY()<=5&&ball.getY()>=-5&&wo>op)
	     return goToTarget(ball);
	     else{
	      if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
           // return SKILL_STAND;
             return SKILL_WALK_OMNI;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
     }
}

SkillType NaoBehavior::Anticollision3(double y,double z)
{
        VecPosition target;
        target =VecPosition(y,ball.getY()+z,0);
        VecPosition center=ball;
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
        //if(ball.getX()>-5)
	    //return goToTarget(target);
	   // else{abs(me.getAngleBetweenPoints(target, ball)) < 90.0
	 //   if (me.getDistanceTo(target) < .25 && abs(me.getAngleBetweenPoints(target, ball)) < 90.0) {
         if(me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
            return SKILL_WALK_OMNI;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
       // }
     }
}

SkillType NaoBehavior::Anticollision4(double x,double y)
{
        double wo=woplayjuli();
        double op=opplayjuli();
        VecPosition target;
        if(ball.getX()<=0&&wo<=op)
        target =VecPosition(0,x+y,0);
        else if(ball.getX()<=0&&wo>op){
        target =VecPosition(x,ball.getY()/1.2,0);
        }else if(ball.getX()>0){
        target =VecPosition(x,ball.getY(),0);
        }
        

        VecPosition center=ball;
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
      
	    if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
            return SKILL_WALK_OMNI;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
     
}

SkillType NaoBehavior::Anticollision5(double x,double y,double z)
{
        VecPosition target;
        if(z==whogo())//倒下补上
        target=ball;
        else
        target =VecPosition(ball.getX()+x,ball.getY()+y,0);
        VecPosition center=ball;
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
        //if(ball.getX()>-5)
	    //return goToTarget(target);
	    //else{abs(me.getAngleBetweenPoints(target, ball)) < 90.0
	    //if (me.getDistanceTo(target) < .25 && abs(me.getAngleBetweenPoints(target, ball)) < 90.0) {
         if(me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
            return SKILL_WALK_OMNI;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
       // }
     }
}

SkillType NaoBehavior::Anticollision6(double x,double y)
{
       VecPosition target;
       target=VecPosition(-HALF_FIELD_X+x,ball.getY()/y,0);
       VecPosition center=ball;
       VecPosition localCenter = worldModel->g2l(center);
       SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
       target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
       if(me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
            return SKILL_WALK_OMNI;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
}

SkillType NaoBehavior::Anticollision7(double x)
{
        VecPosition target;
        target=VecPosition(ball.getX()+6,8*x,0);
        VecPosition center=ball;
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
        if(me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
            return SKILL_WALK_OMNI;
         } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
         } else {
            // Move toward target location
            return goToTarget(target);
         }
}

SkillType NaoBehavior::Anticollision8(double x){
    VecPosition target;
    VecPosition wogoal=VecPosition(-15,0,0);
    double k= (ball.getY()-wogoal.getY())/(ball.getX()-wogoal.getX());
    double b= wogoal.getY()-k*wogoal.getX();
    double y=k*x+b;
    target=VecPosition(x,y,0);
    VecPosition center=ball;
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, false/*keepDistance*/);
        if(me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            //return SKILL_STAND;
            return SKILL_WALK_OMNI;
         } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
         } else {
            // Move toward target location
            return goToTarget(target);
         }
}



SkillType NaoBehavior::demoKickingCircle() 
{
    // Parameters for circle
    VecPosition center = VecPosition(-HALF_FIELD_X/2.0, 0, 0);
    VecPosition goal= VecPosition(HALF_FIELD_X,-0.5,0);
    double circleRadius = 2.0;
    double rotateRate = 2.5;

    
    if (belong() == worldModel->getUNum()) 
    {
        // Have closest player kick the ball toward the center
        return kickBall(KICK_IK, goal);
    } 
    else 
    {
        // Move to circle position around center and face the center
        VecPosition a = VecPosition(ball.getX()-2,ball.getY(),ball.getZ());
        SIM::AngDeg localCenterAngle = atan2Deg(goal.getY(), goal.getX());

        // Our desired target position on the circle
        // Compute target based on uniform number, rotate rate, and time
        VecPosition target = a + VecPosition(circleRadius,0,0).rotateAboutZ(360.0/(6)*(worldModel->getUNum()-(worldModel->getUNum() > belong() ? 1 : 0)) + worldModel->getTime()*rotateRate);

        // Adjust target to not be too close to teammates or the ball
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);
        if(worldModel->getFallenTeammate(belong())==1&&me.getDistanceTo(ball)<2)
        return kickBall(KICK_IK, goal);
        if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) 
        {
            // Close enough to desired position and orientation so just stand
            return SKILL_STAND;
        } else if (me.getDistanceTo(target) < .5) 
        {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else 
        {
            // Move toward target location
            return goToTarget(target);
        }
    }
}

SkillType NaoBehavior::getWalk(const double& direction, const double& rotation, double speed, bool fAllowOver180Turn)
{
    return getWalk(WalkRequestBlock::PARAMS_DEFAULT, direction, rotation, speed, fAllowOver180Turn);
}

SkillType NaoBehavior::getWalk(WalkRequestBlock::ParamSet paramSet, const double& direction, double rotation, double speed, bool fAllowOver180Turn)
{
    double reqDirection, relSpeed;

    if (worldModel->getTime()-lastGetupRecoveryTime < 1.0 && abs(direction) > 90) {
        // Don't try and walk backwards if we just got up as we are probably unstable
        speed = 0;
    }

    // Convert direction angle to the range [0, 360)
    reqDirection = fmod(direction, 360.0);
    reqDirection += (reqDirection < 0) ? 360 : 0;
    assert((reqDirection >= 0) && (reqDirection <= 360));

    // Trim the relative speed
    relSpeed = trim(speed, 0, 1);

    double tanReqDirection, tanMaxSpeed;
    double maxSpeedX, maxSpeedY, maxRot;

    // Desired velocity and rotation as a percentage of the maximum speed.
    double relSpeedX, relSpeedY, relRot;

    // Get the maximum speed.
    maxSpeedX = core->motion_->getMaxXSpeed(); //core->walkEngine.p.speedMax.translation.x;
    maxSpeedY = core->motion_->getMaxYSpeed(); // core->walkEngine.p.speedMax.translation.y;

    relRot = rotation;
    // There is no reason to request a turn > 180 or < -180 as in that case
    // we should just turn the other way instead
    if (!fAllowOver180Turn) {
        if (relRot > 180) {
            relRot -= 360.0;
        } else if (relRot < -180) {
            relRot += 360.0;
        }
    }

    relRot = rotation / 180;

    // Truncate to (+/-)1.0
    relRot = trim(relRot, -1, 1);

    // Calculate tangent. Due to floating point error and the special way the walk
    // engine treats walk requests with only one non-zero component, it is necessary
    // to explicitly set values for direction requests that are multiples of 90.
    if ((reqDirection == 0) || (reqDirection == 180))
        tanReqDirection = 0;
    else if ((reqDirection == 90) || (reqDirection == 270))
        tanReqDirection = INFINITY;
    else
        tanReqDirection = abs(tanDeg(reqDirection));

    tanMaxSpeed = maxSpeedY / maxSpeedX;

    // Determine the maximum relative speeds that will result in
    // a walk in the appropriate direction.
    if (tanReqDirection < tanMaxSpeed)
    {
        relSpeedX = 1;
        relSpeedY = tanReqDirection / tanMaxSpeed;
    }
    else
    {
        relSpeedX = tanMaxSpeed / tanReqDirection;
        relSpeedY = 1;
    }

    // Get signs correct. Forward is positive X. Left is positive Y.
    if (reqDirection > 180)
        relSpeedY *= -1;

    if ((reqDirection > 90) && (reqDirection < 270))
        relSpeedX *= -1;

    // Abrubt stops or changes in direction can be unstable with the approach
    // ball walk parameter set so check for this and stabilize if need be
    static WalkRequestBlock::ParamSet lastWalkParamSet = WalkRequestBlock::PARAMS_DEFAULT;
    static bool fLastWalkParamRequestWasApproach = false;
    static double lastWalkParamRequestApproachTime = 999999999;
    bool fStabilize = false;
    if (paramSet == WalkRequestBlock::PARAMS_APPROACH_BALL) {
        if (!fLastWalkParamRequestWasApproach) {
            lastWalkParamRequestApproachTime = worldModel->getTime();
        }
        fLastWalkParamRequestWasApproach = true;

        if (lastWalkParamSet != WalkRequestBlock::PARAMS_APPROACH_BALL && (speed < .5 || abs(direction) > 45)) {
            if (worldModel->getTime()-lastWalkParamRequestApproachTime < .5) {
                paramSet = WalkRequestBlock::PARAMS_DEFAULT;
                fStabilize = true;
                relSpeed, relRot = 0;
            }
        }
    } else {
        fLastWalkParamRequestWasApproach = false;
    }

    if (lastWalkParamSet != paramSet) {
        lastWalkParamSet = paramSet;
    }


    // Sanity checks. The absolute value of these variables must be <= 1.
    // However, because of floating point error, it's possible that they are
    // slightly greater than one.
    assert(abs(relSpeedX) < 1.001);
    assert(abs(relSpeedY) < 1.001);
    assert(abs(relRot) < 1.001);


    // Record the desired velocity and return the SKILL_WALK_OMNI.
    // NaoBehavior::act() will use the speed components in velocity
    // generate a request to the omnidirectional walk engine whenever the
    // SKILL_WALK_OMNI is invoked.
    velocity = WalkVelocity(paramSet, relSpeed * relSpeedX, relSpeed * relSpeedY, relRot);

    if (fStabilize) {
        // Stabilize
        return SKILL_STAND;
    }

    return SKILL_WALK_OMNI;
}

// Currently untuned. For example, there's no slow down...
SkillType NaoBehavior::goToTargetRelative(const VecPosition& targetLoc, const double& targetRot, const double speed, bool fAllowOver180Turn, WalkRequestBlock::ParamSet paramSet)
{
    double walkDirection, walkRotation, walkSpeed;

    walkDirection = targetLoc.getTheta();
    walkRotation = targetRot;
    walkSpeed = speed;

    walkSpeed = trim(walkSpeed, 0.1, 1);

    if (targetLoc.getMagnitude() == 0)
        walkSpeed = 0;

    return getWalk(paramSet, walkDirection, walkRotation, walkSpeed, fAllowOver180Turn);
}


//Assumes target = z-0. Maybe needs further tuning
SkillType NaoBehavior::goToTarget(const VecPosition &target) {
    double distance, angle;
    getTargetDistanceAndAngle(target, distance, angle);

    const double distanceThreshold = 1;
    const double angleThreshold = getLimitingAngleForward() * .9;
    VecPosition relativeTarget = VecPosition(distance, angle, 0, POLAR);

    // Turn to the angle we want to walk in first, since we want to walk with
    // maximum forwards speeds if possible.
    /*if (abs(angle) > angleThreshold)
    {
    return goToTargetRelative(VecPosition(), angle);
    }*/

    // [patmac] Speed/quickness adjustment
    // For now just go full speed in the direction of the target and also turn
    // toward our heading.
    SIM::AngDeg turnAngle = angle;

    // If we are within distanceThreshold of the target, we walk directly to the target
    if (me.getDistanceTo(target) < distanceThreshold) {
        turnAngle = 0;
    }

    // Walk in the direction that we want.
    return goToTargetRelative(relativeTarget, turnAngle);
}

double NaoBehavior::getLimitingAngleForward() {
    double maxSpeedX = core->motion_->getMaxXSpeed(); //core->walkEngine.p.speedMax.translation.x;
    double maxSpeedY = core->motion_->getMaxYSpeed(); // core->walkEngine.p.speedMax.translation.y;
    return abs(atan2Deg(maxSpeedY, maxSpeedX));
}


void NaoBehavior::refresh() {
    myXDirection = worldModel->l2g(VecPosition(1.0, 0, 0)) - worldModel->l2g(VecPosition(0, 0, 0));
    myXDirection.setZ(0);
    myXDirection.normalize();

    myYDirection = worldModel->l2g(VecPosition(0, 1.0, 0)) - worldModel->l2g(VecPosition(0, 0, 0));
    myYDirection.setZ(0);
    myYDirection.normalize();

    //Anomalous
    myZDirection = worldModel->l2g(VecPosition(0, 0, 1.0)) - worldModel->l2g(VecPosition(0, 0, 0));
    myZDirection.normalize();

    me = worldModel->getMyPosition(); // ->l2g(VecPosition(0, 0, 0)); // <- had consistency problems
    me.setZ(0);

    ball = worldModel->getBall();
    ball.setZ(0);
}

SkillType NaoBehavior::posit() 
{
     switch(worldModel->getUNum())
     {
       case 11:
       return goToTarget(VecPosition(8,5,0));
       case 10:
       return goToTarget(VecPosition(8,-5,0));
       case 9:
       return SKILL_WALK_OMNI;
       case 8:
       return goToTarget(VecPosition(12.6,0,0));
       case 7:
       return goToTarget(VecPosition(4,2,0));
       case 6:
       return goToTarget(VecPosition(4,-2,0));
       case 5:
       return goToTarget(VecPosition(6,0,0));
       case 4:
       return goToTarget(VecPosition(4,0,0));
       case 3:
       return goToTarget(VecPosition(-HALF_FIELD_X+5,ball.getY()/2,0));
       case 2:
       return goToTarget(VecPosition(-HALF_FIELD_X+3,ball.getY()/4,0));
       case 1:
       return goToTarget(VecPosition(-HALF_FIELD_X+1,0,0));
     }
}

SkillType NaoBehavior::posit1() 
{
     switch(worldModel->getUNum())
     {
       case 11:
       return goToTarget(VecPosition(ball.getX(),ball.getY()*0.95,0));
       case 10:
       return goToTarget(VecPosition(ball.getX()*0.95,ball.getY()*0.9,0));
       case 9:
       return SKILL_WALK_OMNI;
       case 8:
       return goToTarget(VecPosition(ball.getX()*0.9,ball.getY()*0.95,0));
       case 7:
       return goToTarget(VecPosition(-12,1.5,0));
       case 6:
       return goToTarget(VecPosition(-12,-1.5,0));
       case 5:
       return goToTarget(VecPosition(-13,-2,0));
       case 4:
       return goToTarget(VecPosition(-13,2,0));
       case 3:
       return goToTarget(VecPosition(-HALF_FIELD_X+5,ball.getY()/2,0));
       case 2:
       return goToTarget(VecPosition(-HALF_FIELD_X+3,ball.getY()/4,0));
       case 1:
       return goToTarget(VecPosition(-HALF_FIELD_X+1,0,0));
     }
}

SkillType NaoBehavior::posit2() 
{
     switch(worldModel->getUNum())
     {
       case 11:
       return goToTarget(VecPosition(-6,-1,0));
       case 10:
       return goToTarget(VecPosition(-7,-1,0));
       case 9:
       return SKILL_WALK_OMNI;
       case 8:
       return goToTarget(VecPosition(-7,1,0));
       case 7:
       return goToTarget(VecPosition(-6,1,0));
       case 6:
       return goToTarget(VecPosition(-8,0,0));
       case 5:
       return goToTarget(VecPosition(-9,0,0));
       case 4:
       return goToTarget(VecPosition(-10,0,0));
       case 3:
       return goToTarget(VecPosition(-HALF_FIELD_X+5,ball.getY()/2,0));
       case 2:
       return goToTarget(VecPosition(-HALF_FIELD_X+3,ball.getY()/4,0));
       case 1:
       return goToTarget(VecPosition(-HALF_FIELD_X+1,0,0));
     }
}

SkillType NaoBehavior::posit3()
{
     double y=ball.getY();
     switch(worldModel->getUNum())
     {
         case 3:
         return goToTarget(VecPosition(-HALF_FIELD_X+5,y/2,0));
         case 2:
         return goToTarget(VecPosition(-HALF_FIELD_X+3,y/4,0));
         case 1:
         return goToTarget(VecPosition(-HALF_FIELD_X+1,0,0));

     }
}

SkillType NaoBehavior::posit4()
{
    double y=ball.getY();
    return goToTarget(VecPosition(0,y,0));
}

SkillType NaoBehavior::posit5(){
    switch(worldModel->getUNum())
     {
       case 11:
       return goToTarget(VecPosition(13,2,0));
       case 10:
       return goToTarget(VecPosition(13,-2,0));
       case 9:
       return SKILL_WALK_OMNI;
       case 8:
       return goToTarget(VecPosition(11,4,0));
       case 7:
       return goToTarget(VecPosition(11,-4,0));
       case 6:
       return goToTarget(VecPosition(9,6,0));
       case 5:
       return goToTarget(VecPosition(9,-6,0));
       case 4:
       return goToTarget(VecPosition(10,0,0));
       case 3:
       return goToTarget(VecPosition(-HALF_FIELD_X+5,ball.getY()/2,0));
       case 2:
       return goToTarget(VecPosition(-HALF_FIELD_X+3,ball.getY()/4,0));
       case 1:
       return goToTarget(VecPosition(-HALF_FIELD_X+1,0,0));
     }
}

SkillType NaoBehavior::posit6(){
    switch(worldModel->getUNum())
     {
       case 11:
       return goToTarget(VecPosition(13,2,0));
       case 10:
       return goToTarget(VecPosition(13,-2,0));
       case 9:
       return SKILL_WALK_OMNI;
       case 8:
       return goToTarget(VecPosition(13,4,0));
       case 7:
       return goToTarget(VecPosition(13,-4,0));
       case 6:
       return goToTarget(VecPosition(9,6,0));
       case 5:
       return goToTarget(VecPosition(9,-6,0));
       case 4:
       return goToTarget(VecPosition(10,0,0));
       case 3:
       return goToTarget(VecPosition(-HALF_FIELD_X+5,ball.getY()/2,0));
       case 2:
       return goToTarget(VecPosition(-HALF_FIELD_X+3,ball.getY()/4,0));
       case 1:
       return goToTarget(VecPosition(-HALF_FIELD_X+1,0,0));
     }
}


//Assumes target it z-0.
void NaoBehavior::getTargetDistanceAndAngle(const VecPosition &target, double &distance, double &angle) {
    VecPosition targetDirection = VecPosition(target) - me;
    targetDirection.setZ(0);

    // distance
    distance = targetDirection.getMagnitude();

    // angle
    targetDirection.normalize();

    angle = VecPosition(0, 0, 0).getAngleBetweenPoints(myXDirection, targetDirection);
    if (isnan(angle)) {
        //cout << "BAD angle!\n";
        angle = 0;
    }
    if(myYDirection.dotProduct(targetDirection) < 0) {
        angle = -angle;
    }
}


bool NaoBehavior::beamablePlayMode() {
    int pm = worldModel->getPlayMode();
    return pm == PM_BEFORE_KICK_OFF || pm == PM_GOAL_LEFT || pm == PM_GOAL_RIGHT;
}

bool NaoBehavior::improperPlayMode() {
    return improperPlayMode(worldModel->getPlayMode());
}

/* Playmodes when we can't touch the ball or game is over (it's not proper to do so) */
bool NaoBehavior::improperPlayMode(int pm) {

    if(pm == PM_BEFORE_KICK_OFF) {
        return true;
    }
    else if(pm == PM_GAME_OVER) {
        return true;
    }
    else if(pm == PM_GOAL_LEFT) {
        return true;
    }
    else if(pm == PM_GOAL_RIGHT) {
        return true;
    }

    if(worldModel->getSide() == SIDE_LEFT) {

        if(pm == PM_KICK_OFF_RIGHT) {
            return true;
        }
        else if(pm == PM_KICK_IN_RIGHT) {
            return true;
        }
        else if(pm == PM_CORNER_KICK_RIGHT) {
            return true;
        }
        else if(pm == PM_GOAL_KICK_RIGHT) {
            return true;
        }
        else if(pm == PM_OFFSIDE_LEFT) {
            return true;
        }
        else if(pm == PM_FREE_KICK_RIGHT) {
            return true;
        }
        else if(pm == PM_DIRECT_FREE_KICK_RIGHT) {
            return true;
        }
        else if(pm == PM_PASS_RIGHT) {
            return true;
        }
    }
    else if(worldModel->getSide() == SIDE_RIGHT) {

        if(pm == PM_KICK_OFF_LEFT) {
            return true;
        }
        else if(pm == PM_KICK_IN_LEFT) {
            return true;
        }
        else if(pm == PM_CORNER_KICK_LEFT) {
            return true;
        }
        else if(pm == PM_GOAL_KICK_LEFT) {
            return true;
        }
        else if(pm == PM_OFFSIDE_RIGHT) {
            return true;
        }
        else if(pm == PM_FREE_KICK_LEFT) {
            return true;
        }
        else if(pm == PM_DIRECT_FREE_KICK_LEFT) {
            return true;
        }
        else if(pm == PM_PASS_LEFT) {
            return true;
        }
    }

    return false;
}

bool NaoBehavior::kickPlayMode() {
    return kickPlayMode(worldModel->getPlayMode());
}

bool NaoBehavior::kickPlayMode(int pm, bool eitherTeam) {
    if(!eitherTeam && improperPlayMode(pm)) {
        return false;
    }

    return pm == PM_CORNER_KICK_LEFT || pm == PM_CORNER_KICK_RIGHT || pm == PM_KICK_IN_LEFT || pm == PM_KICK_IN_RIGHT || pm == PM_FREE_KICK_LEFT || pm == PM_FREE_KICK_RIGHT || pm == PM_DIRECT_FREE_KICK_LEFT || pm == PM_DIRECT_FREE_KICK_RIGHT || pm == PM_GOAL_KICK_LEFT || pm == PM_GOAL_KICK_RIGHT || pm == PM_KICK_OFF_LEFT || pm == PM_KICK_OFF_RIGHT;
}

bool NaoBehavior::isIndirectKick() {
    return isIndirectKick(worldModel->getPlayMode());
}

bool NaoBehavior::isIndirectKick(int pm) {
    if(!kickPlayMode(pm, true)) {
        return false;
    }

    return !(pm == PM_DIRECT_FREE_KICK_LEFT || pm == PM_DIRECT_FREE_KICK_RIGHT || pm == PM_CORNER_KICK_LEFT || pm == PM_CORNER_KICK_RIGHT || pm == PM_GOAL_KICK_LEFT || pm == PM_GOAL_KICK_RIGHT);
}


/* Set message to be send to the monitor port */
void NaoBehavior::setMonMessage(const std::string& msg) {
    monMsg = msg;
}

/* Get message to be sent to the monitor port.  Also flushes message */
string NaoBehavior::getMonMessage() {
    string ret = monMsg;
    monMsg = "";
    return ret;
}



