# Install machine learning packages
chmod +x /opt/mesh_com/modules/utils/package/ml_packages.tar.gz
tar -C /opt/mesh_com/modules/utils/package/ -zxvf /opt/mesh_com/modules/utils/package/ml_packages.tar.gz

cd /opt/mesh_com/modules/utils/package/ml_packages || exit

for f in {*.whl,*.gz};
do
  name="$(echo "$f" | cut -d'-' -f1)"
  if python -c 'import pkgutil; exit(not pkgutil.find_loader("$name"))'; then
    echo "$name" "installed"
  else
    echo "$name" "not found"
    echo "installing" "$name"
    pip install --no-index "$f" --find-links .;
fi
done;

