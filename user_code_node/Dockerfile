
FROM pablogn/rpi-ros-core-indigo

EXPOSE 11311
# setup entrypoint
ENV ROS_DISTRO indigo
COPY ./ros_entrypoint.sh /
USER root
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python", "/combined_code.py"]
COPY ./combined_code.py /