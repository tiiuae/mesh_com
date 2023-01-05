import sys
sys.path.append('../')
from SpectralMgr import Spectral

if __name__ == "__main__":
    spec = Spectral()
    spec.initialize_scan()
    spec.execute_scan()
    f = spec.file_open("/tmp/data")
    spec.read(f, 8512)
    spec.file_close(f)

    print("all values:")
    print(spec.get_values())
