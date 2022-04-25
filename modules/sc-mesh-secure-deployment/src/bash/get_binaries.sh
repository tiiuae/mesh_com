#!/bin/bash
# apt install shc -y

### Check if a directory does not exist ###
[ ! -d "bin" ] && mkdir -p "bin"

echo 'Creating configure binary'
/bin/shc -f ../../configure.sh -o bin/configure

#pip install pyinstaller
#sudo apt-get install python3.9-dev
echo 'Creating server_mesh binary'
pyinstaller ../server_mesh.py --onefile --distpath bin/ --workpath /tmp --clean   --log-level ERROR --key=ssrcencryptionzerotrust
echo 'Creating client_mesh binary'
pyinstaller ../client_mesh.py --onefile --distpath bin/ --workpath /tmp --clean  --log-level ERROR --key=ssrcencryptionzerotrust
echo 'Creating primitives binary'
pyinstaller ../primitives.py --onefile --distpath bin/ --workpath /tmp --clean --log-level ERROR --key=ssrcencryptionzerotrust

rm *.spec