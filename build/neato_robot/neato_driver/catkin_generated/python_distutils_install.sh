#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/src/neato_robot/neato_driver"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install/lib/python2.7/dist-packages:/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build" \
    "/usr/bin/python" \
    "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/src/neato_robot/neato_driver/setup.py" \
    build --build-base "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/neato_robot/neato_driver" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install" --install-scripts="/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install/bin"
