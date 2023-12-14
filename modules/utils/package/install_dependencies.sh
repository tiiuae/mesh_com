# Install torch
chmod +x /opt/mesh_com/modules/utils/package/ml_packages.tar.gz
tar -C /opt/mesh_com/modules/utils/package/ -zxvf /opt/mesh_com/modules/utils/package/ml_packages.tar.gz
cd /opt/mesh_com/modules/utils/package/ml_packages || exit

for f in {*.whl,*.gz}
do
  package="$(echo "$f" | cut -d'-' -f1)"
  if python -c 'import pkgutil; exit(not pkgutil.find_loader("$package"))'; then
    echo "$package installed."
  else
    echo "$package not found. Installing..."
    pip install --no-index "$f" --find-links .;
  fi
done

# Install pypcap
cp /opt/mesh_com/modules/utils/package/ml_packages/libpcap.so.0.8 /usr/lib/.
cp /opt/mesh_com/modules/utils/package/ml_packages/pcap.cpython-39-aarch64-linux-gnu.so /usr/lib/python3.9/site-packages/.

# Install scipy, scikit-learn
packages=("scipy" "scikit_learn")

for package in "${packages[@]}"
do
    if python -c 'import pkgutil; exit(not pkgutil.find_loader("$package"))'; then
      echo "$package installed."
    else
      chmod +x "/opt/mesh_com/modules/utils/package/${package}.tar.gz"
      tar -C /opt/mesh_com/modules/utils/package/ -zxvf "/opt/mesh_com/modules/utils/package/${package}.tar.gz"
      cd "/opt/mesh_com/modules/utils/package/${package}" || exit
      cp -r "/opt/mesh_com/modules/utils/package/${package}"/* "/usr/lib/python3.9/site-packages/"
    fi
done
