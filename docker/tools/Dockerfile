from evo/base

MAINTAINER Kazuya Otani, <kazuotani14@gmail.com>

USER root

# Install some basic GUI and sound libs
RUN apt-get update
RUN apt-get install -y \
	xz-utils file locales dbus-x11 pulseaudio dmz-cursor-theme \
	fonts-dejavu fonts-liberation hicolor-icon-theme \
	libcanberra-gtk3-0 libcanberra-gtk-module libcanberra-gtk3-module \
	libasound2 libgtk2.0-0 libdbus-glib-1-2 libxt6 libexif12 \
	libgl1-mesa-glx libgl1-mesa-dri \
	&& update-locale LANG=C.UTF-8 LC_MESSAGES=POSIX

# Intall some basic GUI tools
RUN apt-get update && apt-get install -y \
	terminator \
	gedit \
	vim \
	nano \
	tmux \
	nemiver \
	chromium-browser \
	qtcreator

# Tools that I (kazu) like to use
RUN add-apt-repository ppa:webupd8team/atom
RUN apt-get update && sudo apt-get install -y atom \
										      git-cola \
										 	  gitk \
										      htop \
											  terminator

USER camerauser
