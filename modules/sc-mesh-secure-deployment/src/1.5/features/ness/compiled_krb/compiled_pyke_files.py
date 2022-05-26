# compiled_pyke_files.py

from pyke import target_pkg

pyke_version = '1.1.1'
compiler_version = 1
target_pkg_version = 1

try:
    loader = __loader__
except NameError:
    loader = None

def get_target_pkg():
    return target_pkg.target_pkg(__name__, __file__, pyke_version, loader, {
         ('', '', 'ness_check.krb'):
           [1634118012.1339242, 'ness_check_bc.py'],
         ('', '', 'ness_fact.kfb'):
           [1634118012.1362686, 'ness_fact.fbc'],
        },
        compiler_version)

