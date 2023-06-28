#pragma once
#include "../behaviors/naobehavior.h"

class Test:public NaoBehavior{
    public:
        Test(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);

    protected:
        virtual void beam( double& beamX, double& beamY, double& beamAngle);
        virtual SkillType selectSkill();
    private:
        int kickType;
        bool kicked;
};