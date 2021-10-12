docker build -t seccomms .
docker run -it --privileged --net="host" --rm seccomms /bin/bash
#docker run -it test /bin/bash
