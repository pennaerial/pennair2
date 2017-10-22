try:
 # for Python2
 from Tkinter import *
except ImportError:
 # for Python3
 from tkinter import *

import PIL.ImageTk


class GroundStationApp(Frame):
    def __init__(self, master=None, TODO=None, DONE=None):
        super().__init__(master)


root = Tk()

w = Label(root, text="Hello, world!")
w.pack()

root.mainloop()


