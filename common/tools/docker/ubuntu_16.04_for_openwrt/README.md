# Docker template for OpenWrt builds

Create work directory and copy template into empty folder.

Following steps in your work directory:

### 1 create ubuntu docker image with image tag name
```
./1_create_docker_image.sh <image_tag_name>
```
### 2 run docker and attach to docker session.  
- my_openwrt_build is freely chosen name for container/host
- image_tag_name is name used in step 1.
```
./2_run_docker.sh <my_openwrt_build> <image_tag_name>
```
[hit enter few times and terminal is open]

### 3 copy your favorite openwrt/. to build/openwrt/. folder 

### 4 start build
```
root@my_openwrt_build:/build/openwrt# ./build_all.sh
```


