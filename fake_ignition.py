#!/usr/bin/env python3
import time
from cereal import log
import cereal.messaging as messaging

def main():
  pm = messaging.PubMaster(['pandaStates'])
  
  print("Publishing fake ignition to pandaStates...")
  while True:
    msg = messaging.new_message('pandaStates', 1)
    msg.pandaStates[0].pandaType = log.PandaState.PandaType.uno
    msg.pandaStates[0].ignitionLine = True
    msg.pandaStates[0].ignitionCan = True
    pm.send('pandaStates', msg)
    time.sleep(0.5)

if __name__ == "__main__":
  main()
