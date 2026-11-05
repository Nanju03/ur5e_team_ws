# Shared environment for manual and unattended Gazebo runs. Source after env.bash.
if [[ -f /usr/share/gazebo-11/setup.sh ]]; then
  source /usr/share/gazebo-11/setup.sh
fi
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11355
export GAZEBO_MODEL_DATABASE_URI=""
unset GTK_PATH GTK_EXE_PREFIX GTK_MODULES GIO_MODULE_DIR GIO_EXTRA_MODULES GTK_IM_MODULE_FILE GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR
export QT_QPA_PLATFORMTHEME=generic QT_STYLE_OVERRIDE=Fusion QT_IM_MODULE=xim GTK_IM_MODULE=xim
gazebo_compat_prefix="$UR5E_WS/compat/gazebo_ros2_control/opt/ros/humble"
if [[ -f "$gazebo_compat_prefix/lib/libgazebo_ros2_control.so" ]]; then
  export AMENT_PREFIX_PATH="$gazebo_compat_prefix:$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$gazebo_compat_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export GAZEBO_PLUGIN_PATH="$gazebo_compat_prefix/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
fi
unset gazebo_compat_prefix
