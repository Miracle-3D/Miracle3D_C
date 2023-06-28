#!/bin/bash
type=0
./agentspark --unum 1 --type $type --paramsfile paramfiles/defaultParams.txt --paramsfile paramfiles/defaultParams_t${type}.txt --optimize test

# --optimize tests

# KICK_FORWARD :7.5
# KICK_DRIBBLE 跑踢
# a