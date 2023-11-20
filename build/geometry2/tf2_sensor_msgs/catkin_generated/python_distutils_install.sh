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

echo_and_run cd "/home/lwt/jackal_ws/src/geometry2/tf2_sensor_msgs"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/lwt/jackal_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/lwt/jackal_ws/install/lib/python2.7/dist-packages:/home/lwt/jackal_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/lwt/jackal_ws/build" \
    "/usr/bin/python2" \
    "/home/lwt/jackal_ws/src/geometry2/tf2_sensor_msgs/setup.py" \
     \
    build --build-base "/home/lwt/jackal_ws/build/geometry2/tf2_sensor_msgs" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/lwt/jackal_ws/install" --install-scripts="/home/lwt/jackal_ws/install/bin"
