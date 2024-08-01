FROM ubuntu:20.04

# RUN rm /etc/apt/sources.list
# COPY Docker/sources.list /etc/apt/sources.list
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update 
RUN apt-get install -y python3-pip 
RUN apt-get install -y openssh-server

ENV DEBIAN_FRONTEND=noninteractive
# dependency for pygame GUI
RUN apt-get install -y libgtk-3-0



WORKDIR /TacticsCarla
COPY PythonAPI /TacticsCarla/PythonAPI
RUN pip3 install ./PythonAPI/carla/dist/carla-0.9.12-cp38-cp38-linux_x86_64.whl


COPY tactics2d/requirements.txt /TacticsCarla/tactics2d/requirements.txt
RUN pip3 install -r tactics2d/requirements.txt

ENV PYTHONPATH="PYTHONPATH:/TacticsCarla/tactics2d"

# Additional dependencies
RUN apt-get install -y python3-tk
RUN pip3 install pytest
RUN pip3 install open3d
RUN apt-get install libgl1-mesa-glx -y

# Create an SSH directory
RUN mkdir /var/run/sshd

# Set the root password
RUN echo 'root:rootpassword' | chpasswd

# Allow the root user to log in via SSH
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# Remove pam restrictions
RUN sed -i 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' /etc/pam.d/sshd

# Expose the port
EXPOSE 22

# Start the SSH service
# CMD ["/usr/sbin/sshd", "-D"]

CMD ["/bin/bash"]


