try:
 # for Python3
 from tkinter import *
except ImportError:
 # for Python2
 from Tkinter import *

try:
    from PIL import ImageTk
    from PIL import Image
    import requests
    import googlemaps
    from staticmap import StaticMap, CircleMarker, Line
except ImportError:
    exit()


class GroundStationApp:
    def __init__(self, master, drone_latitude, drone_longitude):
        self.master = master
        self.map_coordinates = (drone_latitude, drone_longitude)
        self.obstacles = []
        self.boundaries = []
        self.altitude_value = 0
        self.speed_value = 0
        self.create_altitude_widget()
        self.create_altitude_value_widget()
        self.create_speed_widget()
        self.create_speed_value_widget()
        self.create_speed_value_widget()
        self.refresh_map()

    def add_boundary(self, start_long, start_lat, end_long, end_lat):
        self.boundaries.append(Line(((start_long, start_lat), (end_long, end_lat)), 'blue', 3))
        self.refresh_map()
        self.master.update()

    def add_obstacle(self, long, lat):
        self.obstacles.append(CircleMarker((long, lat), 'red', 12))
        self.refresh_map()
        self.master.update()

    def refresh_map(self):
        self.render_image()
        self.display_image()

    def display_image(self):
        self.current_image_path = "map.png"
        self.img = Image.open(self.current_image_path)
        self.photo_img = ImageTk.PhotoImage(self.img)
        self.map_widget = Label(self.master, image=self.photo_img)
        self.map_widget.grid(row=1, column=0, columnspan=4)

    def create_altitude_widget(self):
        self.altitude_widget = Label(self.master, text="Altitude:")
        self.altitude_widget.config(font=("Arial", 24))
        self.altitude_widget.grid(row=0, column=0)

    def create_altitude_value_widget(self):
        self.altitude_value_widget = Label(self.master, text=str(self.altitude_value))
        self.altitude_value_widget.config(font=("Arial", 24))
        self.altitude_value_widget.grid(row=0, column=1)

    def create_speed_widget(self):
        self.speed_widget = Label(self.master, text="Speed:")
        self.speed_widget.config(font=("Arial", 24))
        self.speed_widget.grid(row=0, column=2)

    def create_speed_value_widget(self):
        self.speed_value_widget = Label(self.master, text=str(self.speed_value))
        self.speed_value_widget.config(font=("Arial", 24))
        self.speed_value_widget.grid(row=0, column=3)


    # This method renders the images
    def render_image(self):
        self.static_map = StaticMap(700, 700)#, url_template='http://a.tile.osm.org/{z}/{x}/{y}.png')
        self.drone_marker = CircleMarker(self.map_coordinates, '#0036FF', 12)
        self.static_map.add_marker(self.drone_marker)
        for obstacle in self.obstacles:
            self.static_map.add_marker(obstacle)
            print("added it")

        self.image = self.static_map.render(zoom=17)
        self.image.save('map.png')

    def run(self):
        self.master.mainloop()


# The main method which runs the code for testing purposes
def main():
    root = Tk()
    app = GroundStationApp(root, -75.165222, 39.952583)
    app.add_boundary(-75.164000, 39.951000, -75.165300, 39.952600)
    app.add_obstacle(-75.164220, 39.9526585)
    app.run()


if __name__ == "__main__":
    main()