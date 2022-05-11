#!/bin/bash
# apt install shc -y

### Check if a directory does not exist ###
[ ! -d "bin" ] && mkdir -p "bin"

echo 'Creating configure binary'
/bin/shc -r -f ../../configure.sh -o bin/configure

#pip install pyinstaller
#sudo apt-get install python3.9-dev
#pip3 install cython
echo 'Creating server_mesh binary'
cp server_mesh.py server_mesh.pyx
cython server_mesh.pyx --embed
gcc -Os -I /usr/include/python3.9 -o server_mesh server_mesh.c -lpython3.9 -lpthread -lm -lutil -ldl

cp client_mesh.py client_mesh.pyx
cython client_mesh.pyx --embed
gcc -Os -I /usr/include/python3.9 -o client_mesh client_mesh.c -lpython3.9 -lpthread -lm -lutil -ldl
#pyinstaller ../server_mesh.py --onefile --distpath bin/ --workpath /tmp --clean   --log-level ERROR --key=ssrcencryptionzerotrust
echo 'Creating client_mesh binary'
#pyinstaller ../client_mesh.py --onefile --distpath bin/ --workpath /tmp --clean  --log-level ERROR --key=ssrcencryptionzerotrust
echo 'Creating primitives binary'
cp primitives.py primitives.pyx
cython primitives.pyx --embed
gcc -Os -I /usr/include/python3.9 -o primitives primitives.c -lpython3.9 -lpthread -lm -lutil -ldl

#pyinstaller ../primitives.py --onefile --distpath bin/ --workpath /tmp --clean --log-level ERROR --key=ssrcencryptionzerotrust

rm *.spec *.pyx *.c

