#include "./test.h"

extern void PutMessage(const string& msg);

Test::Test(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_)
        :NaoBehavior(teamName,uNum,namedParams_,rsg_){
        this->kickType = KICK_FORWARD;
        this->kicked = false;
}

void Test::beam( double& beamX, double& beamY, double& beamAngle){
    beamX = -1;
    beamY = 0;
    beamAngle = 0;
}

SkillType Test::selectSkill(){
    int play_mode = worldModel->getPlayMode();
    // VecPosition me = worldModel->getMyPosition();
    // VecPosition ball = worldModel->getBall();
    static int countStep = 0;
    countStep++;


    if(play_mode == PM_PLAY_ON){
        return kickBall(KICK_IK,VecPosition(15,0));
        //return goToTarget(VecPosition(15,0));
    }
       
    
    return SKILL_STAND;
}