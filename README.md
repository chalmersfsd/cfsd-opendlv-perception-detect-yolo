# OpenDLV microservice to detect objects in a video feed by using Yolo (optional GPU acceleration)

## Usage

``
nvidia-docker run -ti --rm --privileged --init --ipc=host --net=host -e DISPLAY=$DISPLAY -v /tmp:/tmp -v ${PWD}/custom.cfg:/opt/yolo.cfg -v ${PWD}/custom.weights:/opt/yolo.weights image_name:version opendlv-perception-detect-yolo --cid=111 --cfg-file=/opt/yolo.cfg --weight-file=/opt/yolo.weights --width=1280 --height=720 --camera=1 --verbose
``

## License

* This project is released under the terms of the GNU GPLv3 License
