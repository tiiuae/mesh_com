certificate:
	if [ ! "$(ls -A cryptolib/)" ]; then git clone https://github.com/tiiuae/cryptolib.git ; cd cryptolib ; make ; cd .. ; fi
	sudo apt-get --yes install clang libssl-dev
	cd cryptolib; make; make cert; cd ..
	cp cryptolib/ecc_key.der src/
	if [ ! -f "src/ecies_decrypt" ]; then cp cryptolib/ecies_decrypt cryptolib/ecies_encrypt src/ ; fi


client:
	if [ ! "$(ls -A cryptolib/)" ]; then  git clone https://github.com/tiiuae/cryptolib.git ; cd cryptolib ; make ; cd .. ; fi
	pip3 install -r requirements/client-requirements.txt
	if [ ! -f "src/ecies_decrypt" ]; then cp cryptolib/ecies_decrypt cryptolib/ecies_encrypt src/ ; fi


server:
	if [ ! "$(ls -A cryptolib/)" ]; then  git clone https://github.com/tiiuae/cryptolib.git ; cd cryptolib ; make ; cd .. ; fi
	pip3 install -r requirements/server-requirements.txt
	if [ ! -f "src/ecies_decrypt" ]; then cp cryptolib/ecies_decrypt cryptolib/ecies_encrypt src/ ; fi
