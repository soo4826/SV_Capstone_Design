#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ailab/Project/07_Mbed_ros/ros_mbed_ws/src/rosserial/rosserial_vex_cortex"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ailab/Project/07_Mbed_ros/ros_mbed_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ailab/Project/07_Mbed_ros/ros_mbed_ws/install/lib/python2.7/dist-packages:/home/ailab/Project/07_Mbed_ros/ros_mbed_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ailab/Project/07_Mbed_ros/ros_mbed_ws/build" \
    "/usr/bin/python2" \
    "/home/ailab/Project/07_Mbed_ros/ros_mbed_ws/src/rosserial/rosserial_vex_cortex/setup.py" \
     \
    build --build-base "/home/ailab/Project/07_Mbed_ros/ros_mbed_ws/build/rosserial/rosserial_vex_cortex" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ailab/Project/07_Mbed_ros/ros_mbed_ws/install" --install-scripts="/home/ailab/Project/07_Mbed_ros/ros_mbed_ws/install/bin"
