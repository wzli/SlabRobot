#/bin/bash
docker_image=espressif/idf:release-v4.3
app=esp32_app
docker run -it --rm --privileged -w /project/firmware/app \
-v $PWD/$app:/project/firmware/app \
-v $PWD/../submodules:/project/submodules \
-v $PWD/../controller:/project/controller \
$docker_image idf.py $@
