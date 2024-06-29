# FROM ros:noetic-ros-base
FROM osrf/ros:noetic-desktop
#--------------------------------------------------------------------------------------------------------------
# apt-get installs
#--------------------------------------------------------------------------------------------------------------

# apt update & upgrade ----------------------------------------------------------------------------------------
RUN pwd \
  && rm -rf /var/lib/apt/lists/* \
  && apt full-upgrade \
  && apt -y update --fix-missing

# install and configure locales -------------------------------------------------------------------------------
RUN pwd \
   && apt -y install locales locales-all
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

# install software properties common --------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install software-properties-common --no-install-recommends

# install build essential -------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install build-essential --no-install-recommends

# install wget ------------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install wget --no-install-recommends

# install cmake -----------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install cmake --no-install-recommends

# install unzip -----------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install unzip --no-install-recommends

# install pip3 ------------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install python3-pip --no-install-recommends

# install python3-venv ----------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install python3-venv --no-install-recommends

# install python3-dev -----------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install python3-dev --no-install-recommends

# install bash-completion -------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install bash-completion --no-install-recommends

# install command-not-found -----------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install command-not-found --no-install-recommends

# install powerline-shell -------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && pip3 install powerline-shell

# install sudo ------------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install sudo --no-install-recommends \
  && echo "Set disable_coredump false" >> /etc/sudo.conf

# install git -------------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get -y install git --no-install-recommends


# install kubectl dependencies --------------------------------------------------------------------------------
  RUN pwd \
  && apt-get update \
  && apt-get -y install apt-transport-https ca-certificates curl --no-install-recommends


# install docker -----------------------------------------------------------------------------------------------
RUN pwd \
  && curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null \
  && apt-get update \
  && apt-get install -y docker-ce docker-ce-cli containerd.io --no-install-recommends

# install gpg-agent -------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y gpg-agent --no-install-recommends

# install clang-tidy ------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y clang-tidy --no-install-recommends

# install clang-format ----------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y clang-format --no-install-recommends

# install uncrustify ------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y uncrustify --no-install-recommends

# install cppcheck --------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y cppcheck --no-install-recommends

# install include-what-you-use --------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y iwyu --no-install-recommends

# install ansible ---------------------------------------------------------------------------------------------
RUN pwd \
  && add-apt-repository --yes --update ppa:ansible/ansible \
  && apt-get update \
  && apt-get install -y ansible --no-install-recommends


# install htop ------------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y htop --no-install-recommends

# install tree ------------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y tree --no-install-recommends

# install psmisc ----------------------------------------------------------------------------------------------
# A package of small utilities that use the proc file-system.
## fuser - Identifies processes using files or sockets
## killall - kills processes by name, e.g. killall -HUP named
## prtstat - prints statistics of a process
## pslog - prints log path(s) of a process
## pstree - shows the currently running processes as a tree
## peekfd - shows the data travelling over a file descriptor
RUN pwd \
  && apt-get update \
  && apt-get install -y psmisc --no-install-recommends

# install texlive-latex-base ----------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y texlive-latex-base --no-install-recommends

# install openssh-client --------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install -y openssh-client --no-install-recommends

# install python3-catkin-tools ---------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install python3-catkin-tools --no-install-recommends -y\
  && apt-get install build-essential --no-install-recommends -y\
  && apt-get install freeglut3-dev --no-install-recommends -y\
  && pip3 install evdev

RUN pwd \
  && apt-get install sshpass --no-install-recommends -y

# install gazebo9 ---------------------------------------------------------------------------------------------
RUN pwd \
  && apt-get update \
  && apt-get install gazebo9 gazebo9-plugin-base --no-install-recommends -y\
  && apt-get install ros-noetic-gazebo-ros-pkgs --no-install-recommends -y\
  && apt-get install ros-noetic-ros-controllers --no-install-recommends -y\
  && apt-get install ros-noetic-urdf-sim-tutorial --no-install-recommends -y\ 
  && apt-get install liburdfdom-tools --no-install-recommends -y 

RUN pwd \
  && apt-get update \
  && apt-get install ros-noetic-teb-local-planner --no-install-recommends -y\
  && apt-get install ros-noetic-smach-ros --no-install-recommends -y

#--------------------------------------------------------------------------------------------------------------
# Create vscode user
#--------------------------------------------------------------------------------------------------------------

# create vscode user ------------------------------------------------------------------------------------------
RUN pwd \
  && useradd -m -s /bin/bash vscode \
  && echo 'vscode ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
  && usermod -aG docker vscode

# As vscode user ----------------------------------------------------------------------------------------------
USER vscode

# Source the setup files and delete the gtest folder ---------------------------------------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/vscode/.bashrc \
  && echo "source /MDP/devel/setup.bash" >> /home/vscode/.bashrc \
  && sudo rm -rf /usr/src/gtest

CMD [ "/bin/bash" ]


