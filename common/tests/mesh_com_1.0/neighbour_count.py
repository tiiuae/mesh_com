import subprocess
def get_macs():
    macs = []
    proc = subprocess.Popen(['batctl', 'n'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    i = 0
    for x in proc.stdout:
        aux = x.split()
        i += 1
        if i > 2:
            macs.append((aux[1]).decode("utf-8"))
    count= len(macs)
    print(len(macs))
if __name__ == '__main__':
    get_macs()
  
