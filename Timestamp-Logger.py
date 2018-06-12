"""when program is run, log the timestamp of when the run button is hit,
3 minutes past when the run button is hit(end of baseline collection),
once more when interaction phase begins, and once when interaction ends
"""
import tkinter
from datetime import datetime
import time
import logging

logging.basicConfig(filename='timestamp.log', level=logging.INFO)  # generates a log file (timestamp.txt)


def logbaselinetime():  # tkinter button function to log to file (the baseline 3 minute period)
    logging.info((datetime.now().time()))  # logs the time
    time.sleep(180.000)  # waits 3 minutes
    logging.info((datetime.now().time()))  # logs time again
    logbasebtn.destroy()  # gets rid of interval based timing button
    logsinglebtn = tkinter.Button(frame,  # used for any timestamps after baseline collection
                                  text="Log",
                                  command=logsingletime)
    logsinglebtn.pack(side=tkinter.LEFT)


def logsingletime():  # this logs a single time point,use for start of interaction and end
    logging.info((datetime.now().time()))  # logs the time


root = tkinter.Tk()
frame = tkinter.Frame(root)
frame.pack()

quitbtn = tkinter.Button(frame,
                         text="QUIT",
                         fg="red",
                         command=quit)
quitbtn.pack(side=tkinter.LEFT)
logbasebtn = tkinter.Button(frame,
                            text="Log",
                            command=logbaselinetime)
logbasebtn.pack(side=tkinter.LEFT)

root.mainloop()
