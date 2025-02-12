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

echo_and_run cd "/home/pi/datn_navbot/src/mpu_6050_driver"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/pi/datn_navbot/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/pi/datn_navbot/install/lib/python3/dist-packages:/home/pi/datn_navbot/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/pi/datn_navbot/build" \
    "/usr/bin/python3" \
    "/home/pi/datn_navbot/src/mpu_6050_driver/setup.py" \
     \
    build --build-base "/home/pi/datn_navbot/build/mpu_6050_driver" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/pi/datn_navbot/install" --install-scripts="/home/pi/datn_navbot/install/bin"
