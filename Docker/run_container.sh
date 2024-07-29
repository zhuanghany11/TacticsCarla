
host_path=$(dirname $(cd "$(dirname $"$0")";pwd))
echo $host_path

sudo docker container rm -f TactiscCarla

xhost +local:docker

XAUTH=/tmp/.docker.xauth
XSOCK=/tmp/.X11-unix

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi


echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."


sudo docker run \
    -it \
    --name TactiscCarla \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --network host \
    -v $XSOCK:$XSOCK \
    -v $XAUTH:$XAUTH \
    -v $host_path/main:/TacticsCarla/main \
    -v $host_path/tactics2d:/TacticsCarla/tactics2d \
    tactics_carla:1.0


# sudo docker run \
#     -it \
#     --name TactiscCarla \
#     --shm-size=512m \
#     --privileged \
#     -e DISPLAY=$DISPLAY \
#     -e QT_X11_NO_MITSHM=1 \
#     -e XAUTHORITY=$XAUTH \
#     --network host \
#     -v $XSOCK:$XSOCK \
#     -v $XAUTH:$XAUTH \
#     -v $host_path/DispatcherServer/src:/Dispatcher/DispatcherServer/src \
#     -v $host_path/data:/Dispatcher/data \
#     -v $host_path/config:/Dispatcher/config \
#     tactics_carla:1.0

# # sudo docker ps -a 

#     # -e DISPLAY -e QT_X11_NO_MITSHM=1 \
#     # -v /tmp/.X11-unix:/tmp/.X11-unix \
#     # -v $HOME/.Xauthority:/root/.Xauthority \
