import subprocess
import time
import os
import signal
import psutil
import gc

#  i - set of parameters
#  j - earthquake
dirEQ  = 1    # 1 = longit. dir , 2 = transverse doir
for i in range(1,2):
    for j in range(1):     
        cmd = "python .\dev\PyCoSimulation\src\orchestrator.py {case} {eq} {dir}".format(case=str(i), eq=str(j), dir=str(dirEQ))  #replace with your command
        cmd = cmd.split()  # replace with your command
        process = subprocess.Popen(cmd)
        pid = process.pid
        process.wait()
        process.kill()
        outputs,errs=process.communicate()

