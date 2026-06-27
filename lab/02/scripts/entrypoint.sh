#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO:-noetic}/setup.bash"

export DISPLAY="${DISPLAY:-:1}"
export VNC_RESOLUTION="${VNC_RESOLUTION:-1280x800}"
export VNC_PASSWORD="${VNC_PASSWORD:-ros}"
export LAB02_LAUNCH="${LAB02_LAUNCH:-tracking_pid lab02_demo.launch rviz:=true}"

mkdir -p "${HOME}/.vnc"
printf "%s\n" "${VNC_PASSWORD}" | vncpasswd -f > "${HOME}/.vnc/passwd"
chmod 600 "${HOME}/.vnc/passwd"

cat > "${HOME}/.vnc/xstartup" <<'EOF'
#!/usr/bin/env bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
xsetroot -solid grey
startxfce4 &
EOF
chmod +x "${HOME}/.vnc/xstartup"

vncserver -kill "${DISPLAY}" >/dev/null 2>&1 || true
rm -f "/tmp/.X${DISPLAY#:}-lock" "/tmp/.X11-unix/X${DISPLAY#:}"
vncserver "${DISPLAY}" -geometry "${VNC_RESOLUTION}" -depth 24 >/tmp/vncserver.log 2>&1

websockify --web=/usr/share/novnc 6080 "localhost:590${DISPLAY#:}" >/tmp/novnc.log 2>&1 &

echo "noVNC desktop: http://localhost:6081/vnc.html"
echo "VNC password: ${VNC_PASSWORD}"

cd /workspace/catkin_ws
echo "Building catkin workspace..."
catkin_make

source /workspace/catkin_ws/devel/setup.bash
echo "Starting lab 02 demo..."

if [ "$#" -eq 0 ]; then
  set -- roslaunch ${LAB02_LAUNCH}
fi

exec "$@"
