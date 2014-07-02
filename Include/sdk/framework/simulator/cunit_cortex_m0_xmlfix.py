#!/usr/bin/python
#nordic-harness-script: cpython
import re,sys,shutil

log_file = sys.argv[1]
shutil.copy(log_file, log_file+ ".bak")
f = open(log_file, "r")
lines = f.readlines()
lines[0] = re.sub(r'^G,cu_exit[\r\n]*$', "", lines[0])
lines[-1] = re.sub(r'LOG OFF[\r\n]*$', "", lines[-1])
f.close()
f = open(log_file, 'w')
for l in lines:
  f.write(l)
f.close()
