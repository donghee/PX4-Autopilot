cd "$(dirname "$0")/../.."

# Extract PX4_AUTOPILOT from dirname of current directory
# e.g. /home/user/workspace/PX4-Autopilot -> PX4-Autopilot
PX4_AUTOPILOT=${PWD##*/}
echo "Building $PX4_AUTOPILOT for VxWorks in Docker container..."
docker exec vxbuild /opt/WindRiver/wrenv.linux -- ~/workspace/$PX4_AUTOPILOT/Tools/vxworks/build_px4.sh $@