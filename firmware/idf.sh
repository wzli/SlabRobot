#/bin/bash
docker_image=espressif/idf:release-v4.3
app=esp32_app
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
docker run -it --rm --privileged -w /project/firmware/app \
-v $SCRIPT_DIR/$app:/project/firmware/app \
-v $SCRIPT_DIR/../submodules:/project/submodules \
-v $SCRIPT_DIR/../libslab:/project/libslab \
$docker_image idf.py $@
