#/bin/bash
version=release-v4.2
project=esp32_app
docker run -it --rm --privileged -v $PWD/$project:/project -w /project espressif/idf:$version idf.py $@
