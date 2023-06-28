LIBS_DIR="./libs"
export LD_LIBRARY_PATH=$LIBS_DIR:$LD_LIBRARY_PATH

type=3

params="--paramsfile paramfiles/defaultParams.txt --paramsfile paramfiles/defaultParams_t${type}.txt"

./agentspark  --unum 1 --type ${type} $params 