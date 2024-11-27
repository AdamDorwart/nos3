#!/bin/bash

# setup.sh
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source $SCRIPT_DIR/env.sh

# Check prerequisites
if [ ! -d $USER_NOS3_DIR ]; then
    echo "Need to run make prep first!"
    exit 1
fi

if [ ! -d $BASE_DIR/cfg/build ]; then
    echo "Need to run make config first!"
    exit 1
fi

# Get user and group IDs
USER_ID=$(id -u $(stat -c '%U' $SCRIPT_DIR/env.sh))
GROUP_ID=$(getent group $(stat -c '%G' $SCRIPT_DIR/env.sh) | cut -d: -f3)

cat > .env << EOF
SCRIPT_DIR=${SCRIPT_DIR}
BASE_DIR=${BASE_DIR}
FSW_DIR=${FSW_DIR}
GSW_BIN=${GSW_BIN}
GSW_DIR=${GSW_DIR}
SIM_DIR=${SIM_DIR}
SIM_BIN=${SIM_BIN}
USER_NOS3_DIR=${USER_NOS3_DIR}
OPENC3_DIR=${OPENC3_DIR}
OPENC3_PATH=${OPENC3_PATH}
GND_CFG_FILE=-f nos3-simulator.xml
SC_CFG_FILE=-f nos3-simulator.xml
USER_ID=${USER_ID}
GROUP_ID=${GROUP_ID}
SC_NUM=sc_1
SC_NETNAME=nos3_sc_1
EOF


# Create required directories
mkdir -p $FSW_DIR/data/{cam,evs,hk,inst}
mkdir -p /tmp/nos3/data/{cam,evs,hk,inst}
mkdir -p /tmp/nos3/uplink

# Copy startup files
cp $BASE_DIR/fsw/build/exe/cpu1/cf/cfe_es_startup.scr /tmp/nos3/uplink/tmp0.so
cp $BASE_DIR/fsw/build/exe/cpu1/cf/sample.so /tmp/nos3/uplink/tmp1.so

# Setup 42 visualization
rm -rf $USER_NOS3_DIR/42/NOS3InOut
cp -r $BASE_DIR/cfg/build/InOut $USER_NOS3_DIR/42/NOS3InOut

# Enable X11 forwarding
xhost +local:*

# Export required environment variables
export GND_CFG_FILE="-f nos3-simulator.xml"
export SC_CFG_FILE="-f nos3-simulator.xml"
export SATNUM=1
export SC_NUM="sc_1"
export SC_NETNAME="nos3_sc_1"

# Launch the system
docker compose -f $SCRIPT_DIR/docker-compose.yml up -d
docker compose -f $SCRIPT_DIR/docker-compose.yml logs -f
