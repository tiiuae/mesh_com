import queue
import threading
import yaml
from features import continuous
from features import MBA
from features.mutual.mutual import mutual
from features import NESS
from features import quarantine


class myThread(threading.Thread):
    def __init__(self, threadID, name, q):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.q = q

    def run(self):
        print(f"Starting {self.name}")
        print(f"Exiting {self.name}")


def launchTherads(threadList):
    queueLock = threading.Lock()
    workQueue = queue.Queue(10)
    threads = []
    # Create new threads
    for threadID, tName in enumerate(threadList, start=1):
        thread = myThread(threadID, tName, workQueue)
        thread.start()
        threads.append(thread)
    # Fill the queue
    queueLock.acquire()
    for word in range(len(threadList)):
        workQueue.put(word)
    queueLock.release()

    # Wait for queue to empty
    while not workQueue.empty():
        pass

    # Notify threads it's time to exit
    exitFlag = 1

    # Wait for all threads to complete
    for t in threads:
        t.join()
    print("Exiting Main Thread")


def readfile():
    with open("features.yaml", "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def initialize(feature):
    match feature:
        case 'MBA':
            mba = MBA()
            return mba
        case 'NESS':
            ness = NESS()
            return ness
        case 'continuous':
            cont = continuous()
            return cont
        case 'mutual':
            mut = mutual.Mutual()
            return mut
        case 'quarantine':
            quara = quarantine()
            return quara


if __name__ == "__main__":
    threadList = []
    features = readfile()
    for index in features:
        if features[index]:
            status = initialize(index)
            threadList.append(index)

    launchTherads(threadList)
