#include "pkbehaviors.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
/////// GOALIE
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

PKGoalieBehavior::
PKGoalieBehavior( const std::string teamName,
                  int uNum,
                  const map<string, string>& namedParams_,
                  const string& rsg_)
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_) {
}

void PKGoalieBehavior::
beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -HALF_FIELD_X+.5;
    beamY = 0;
    beamAngle = 0;
}




SkillType PKGoalieBehavior::
selectSkill() {
    double sjs=GetFand();
    if(ball.getX()>-7){
        goToTarget(VecPosition(-14.5,sjs,0));
    }else{
        if(ball.getY()>0){
            return SKILL_LEFT_BLOCK; 
        }else if(ball.getY()<0){
            return SKILL_RIGHT_BLOCK;
        }else{
            return SKILL_MIDDLE_BLOCK;
        }
    }
}





////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
/////// SHOOTER
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
PKShooterBehavior::
PKShooterBehavior( const std::string teamName,
                   int uNum,
                   const map<string, string>& namedParams_,
                   const string& rsg_ )
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_ ) {
}

void PKShooterBehavior::
beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -0.19897997275908952;
    beamY =-0.08550413847290347;
    beamAngle = 0;
}

SkillType PKShooterBehavior::
selectSkill() {
    VecPosition target = VecPosition(15,0,0);
    double jsj = GetFand();
    if(me.getDistanceTo(target)<=7){
        return kickBall(KICK_LONGASS,VecPosition(15,0.3,0));
    }else{
        return kickBall(KICK_DRIBBLE,VecPosition(15,jsj,0));
    }
}
