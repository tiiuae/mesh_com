import os
from features.mutual.utils import primitives as pri
from features.mutual.utils import wifi_ssrc as wf
import sys

import multiprocessing
import queue
import random
from time import time, sleep
import socket
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import threading
import glob
import subprocess
import asyncio

import contextlib
import pandas as pd
import numpy as np
from features.utils import utils as ut
import yaml

from common import ConnectionMgr
from common import mesh_utils
from features.continuous import ca_main
from features.continuous import utils as ca_utils
from features.mba import mba
from features.mutual import mutual
from features.ness import ness_main
from features.quarantine import quarantine

co = ConnectionMgr.ConnectionMgr()
_executor = ThreadPoolExecutor()
executor = ProcessPoolExecutor()
MUTUALINT = 'wlan1'
MESHINT = 'bat0'
client_q = {}
ness = ness_main.NESS()
qua = quarantine.Quarantine()

SEC_BEAT_TIME = 120  # Security beat period in seconds
END_TIME_SEC_BEAT = 1200  # Total time to run security beat in seconds
