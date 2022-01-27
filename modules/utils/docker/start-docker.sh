docker build -t seccomms .
docker run -it --privileged --net="host"  -v /hsm:/opt/mesh_com/modules/sc-mesh-secure-deployment/src/authentication/hsm  --rm seccomms /bin/bash
