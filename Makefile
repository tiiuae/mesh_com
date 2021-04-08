
#root folders
root_dir:=$(shell pwd)
common_root:=$(root_dir)/common
module_root:=$(root_dir)/modules

#common folder
scripts_root:=$(common_root)/scripts
cryptolib_root:=$(common_root)/cryptolib

#module folder
meshcom_root:=$(module_root)/mesh_com
mesh_tb_root:=$(module_root)/sc-mesh-secure-deployment


mesh_com_bloom: #TODO
	cd $(module_root)/mesh_com
	bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy && fakeroot debian/rules binary && mv ../*.deb ../../../packaging/

client:
	pip3 install -r $(mesh_tb_root)/requirements/client-requirements.txt
	if [ ! -f "$(cryptolib_root)/ecies_decrypt" ]; then \
		cp $(cryptolib_root)/ecies_decrypt $(cryptolib_root)/ecies_encrypt $(mesh_tb_root)/src/ ; \
	fi


server:
	sudo apt-get --yes install clang libssl-dev
	cd $(cryptolib_root); make; make cert; cd $(root_dir);
	cp $(cryptolib_root)/ecc_key.der $(mesh_tb_root)/src/;
	if [ ! -f "$(mesh_tb_root)/src/ecies_decrypt" ]; then \
		cp $(cryptolib_root)/ecies_decrypt $(cryptolib_root)/ecies_encrypt $(mesh_tb_root)/src/ ; \
	fi
	pip3 install -r $(mesh_tb_root)/requirements/server-requirements.txt
