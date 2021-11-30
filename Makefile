
#root folders
root_dir:=$(shell pwd)
common_root:=$(root_dir)/common
module_root:=$(root_dir)/modules

#common folder
scripts_root:=$(common_root)/scripts
cryptolib_root:=$(common_root)/security/cryptolib

#module folder
meshcom_root:=$(module_root)/mesh_com
mesh_tb_root:=$(module_root)/sc-mesh-secure-deployment


mesh_com_bloom:
	cd $(module_root)/mesh_com
	bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro galactic && fakeroot debian/rules binary
	cd $(root_dir)

mesh_tb_client:
	pip3 install -r $(mesh_tb_root)/requirements/client-requirements.txt
	if [ ! "$(ls -A $(cryptolib_root))"  ]; then \
		cd $(cryptolib_root); make; make cert; cd $(root_dir); \
	fi
	if [ ! -f "$(mesh_tb_root)/src/ecies_decrypt" ]; then \
		cp $(cryptolib_root)/ecies_decrypt $(cryptolib_root)/ecies_encrypt $(mesh_tb_root)/src/ ; \
	fi

mesh_tb_server:
	sudo apt-get --yes install clang libssl-dev
	pip3 install -r $(mesh_tb_root)/requirements/server-requirements.txt
	if [ ! -f "$(cryptolib_root)/src/ecies_decrypt" ]; then \
		cd $(cryptolib_root); make; make cert; cd $(root_dir); \
	fi
	if [ ! -f "$(mesh_tb_root)/src/ecies_decrypt" ]; then \
		cp $(cryptolib_root)/ecies_decrypt $(cryptolib_root)/ecies_encrypt $(mesh_tb_root)/src/ ; \
	fi
	cp $(cryptolib_root)/ecc_key.der $(mesh_tb_root)/src/;
	# cd $(mesh_tb_root); sudo bash $(mesh_tb_root)/configure.sh -s
