import subprocess
import time
import os
import signal
import psutil
import gc

#  i - length of set of parameters
#  j - number of earthquakes in each set (10 earthquake intensities)
dirEQ  = 1    # 1 = longit. dir , 2 = transverse dir
for i in range(1,100):
    for j in range(1,10):     
        cmd = "python .\dev\PyCoSimulation\src\orchestrator.py {case} {eq} {dir}".format(case=str(i), eq=str(j), dir=str(dirEQ))  
        cmd = cmd.split() 
        process = subprocess.Popen(cmd)
        pid = process.pid
        process.wait()
        process.kill()
        outputs,errs=process.communicate()

