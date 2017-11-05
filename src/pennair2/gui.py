try:
 # for Python2
 from Tkinter import *
except ImportError:
 # for Python3
 from tkinter import *

from PIL import ImageTk
from PIL import Image


class GroundStationApp:
    def __init__(self, master):
        self.master = master
        self.current_image_path = "monkey.jpg"
        self.load_image()

    def load_image(self):
        self.current_image_path = "monkey.jpg"
        img = Image.open(self.current_image_path)
        photo_img = ImageTk.PhotoImage(img)
        panel = Label(self.master, image=photo_img)
        panel.pack(side="bottom", fill="both", expand="yes")

    def run(self):
        self.master.mainloop()

# The main method which runs the code
def main():
    root = Tk()
    app = GroundStationApp(root)
    app.run()


if __name__ == "__main__":
    main()