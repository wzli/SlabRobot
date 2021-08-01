#/bin/bash
docker_image=espressif/idf:release-v4.3
app=esp32_app
SCRIPT_DIR=$(dirname $(readlink -f $0))
docker run -it --rm --privileged -w /project/firmware/app \
-v $SCRIPT_DIR/$app:/project/firmware/app \
-v $SCRIPT_DIR/../submodules:/project/submodules \
-v $SCRIPT_DIR/../libslab:/project/libslab \
$docker_image idf.py $@
