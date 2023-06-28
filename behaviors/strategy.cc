#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"

extern int agentBodyType;

void NaoBehavior::beam( double& beamX, double& beamY, double& beamAngle ) {
        switch(worldModel->getUNum())
  {
    case 1:
      beamX=-14;beamY=0; beamAngle = 0;break;
    case 2: 
      beamX=-13.5;beamY=-5; beamAngle = 0;break;
    case 3:
      beamX=-13.5;beamY=-7; beamAngle = 0;break; 
    case 4:
      beamX=-11;beamY=5; beamAngle = 0;break;
    case 5:
      beamX=-6;beamY=0; beamAngle = 0;break;
    case 6:
      beamX=-2;beamY=-4; beamAngle = 0;break;
    case 7:
      beamX=-6;beamY=6;  beamAngle = 0;;break;
    case 8:
      beamX=-6;beamY=-6; beamAngle = 0;break;
    case 9:
      beamX=-6;beamY=-3; beamAngle = 0;break;
    case 10:
      beamX=-3;beamY=-3; beamAngle = 0;break;
    case 11:
      beamX=-2.5;beamY=0; beamAngle = 0;break;
  }
   
}


SkillType NaoBehavior::selectSkill()
{
  return def();
}
