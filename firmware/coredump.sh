#/bin/bash
docker_image=espressif/idf:release-v4.3
app=esp32_app
core_file=build/$(date +%s).core
SCRIPT_DIR=$(dirname $(readlink -f $0))
docker run -it --rm --privileged -w /project/firmware/app \
-v $SCRIPT_DIR/$app:/project/firmware/app \
$docker_image /bin/bash -c "espcoredump.py info_corefile -s $core_file build/$app.elf \
&& read -p \"Press enter to start gdb\" \
&& espcoredump.py dbg_corefile -c $core_file build/$app.elf"
