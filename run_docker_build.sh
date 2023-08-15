docker run -it --rm -v .:/ws -v /dev:/dev --net=host --privileged idf-build /bin/bash -c "idf.py menuconfig build flash"
