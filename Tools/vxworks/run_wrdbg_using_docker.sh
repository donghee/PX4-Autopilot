cd "$(dirname "$0")/../.."

PX4_AUTOPILOT=${PWD##*/}
echo "Debugging $PX4_AUTOPILOT for VxWorks in Docker container..."
docker exec -it vxbuild /opt/WindRiver/wrenv.linux -- ~/workspace/$PX4_AUTOPILOT/Tools/vxworks/run_wrdbg.sh