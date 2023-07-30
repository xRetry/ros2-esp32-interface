docker run --rm -v .:/ws -v /dev:/dev --net=host idf-build /bin/bash -c "idf.py menuconfig build flash monitor"
