FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update && sudo apt-get install -y \
    ros-indigo-usb-cam \
    ros-indigo-uvc-camera \
    ros-indigo-compressed-image-transport \
&& sudo rm -rf /var/lib/apt/lists/*

USER root
#CMD ["rosrun", "usb_cam", "usb_cam_node", "~video_device", "/dev/video0"]
CMD ["rosrun", "uvc_camera", "uvc_camera_node"]
