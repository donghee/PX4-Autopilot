# Go to the PX4-Autopilot root directory
cd "$(dirname "$0")/../.."

# Set up the debug init file
cp Tools/vxworks/wrdbginit build/yp_fc-v1_default/bin/
cd build/yp_fc-v1_default/bin

# Run the Wind River debugger with the init file
~/WindRiver/wrenv.linux -- wrdbg -x wrdbginit