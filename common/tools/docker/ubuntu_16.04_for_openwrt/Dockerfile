#base ubuntu 16.04
FROM ubuntu:16.04

RUN apt-get update

# Tools installation for build and other utilities
RUN apt-get install -y gcc g++ binutils patch autoconf libcurl4-openssl-dev bzip2 flex make gettext pkg-config unzip zlib1g-dev libc6-dev subversion libncurses5-dev gawk sharutils curl libxml-parser-perl python-yaml git  ocaml-nox libssl-dev repo wget nano python3

RUN  git config --global user.email "dont@contact.us"
RUN  git config --global user.name "JohnDoe"

# Install ccache
RUN apt-get install -y apt-utils ccache
ENV PATH /usr/lib/ccache:$PATH
ENV CONFIG_CCACHE y
ENV WORKSPACE /build

#build folder visible between docker and host
ADD ./build /build

