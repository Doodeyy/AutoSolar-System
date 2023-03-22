import tkinter as tk
import paho.mqtt.client as mqtt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import json

# Define the MQTT broker and topics to subscribe to
mqtt_broker = "localhost"
mqtt_port = 1883
mqtt_topic_battery_amp = "sensors/battery/amp"
mqtt_topic_solar_volt = "sensors/solar/volt"
mqtt_topic_battery_volt = "sensors/battery/volt"

# Define the colors to use for each data source
colors = {
    "battery_amp": "r",
    "solar_volt": "g",
    "battery_volt": "b",
}

# Define the update interval for the graphs (in milliseconds)
update_interval = 1000

def update_graphs():
    # Create the tkinter window
    root = tk.Tk()
    root.title("Solar Panel Monitor")

    # Create the figure and subplots for the graphs
    fig, (ax1, ax2, ax3) = Figure(figsize=(8, 6), dpi=100, tight_layout=True, nrows=3, ncols=1, sharex=True)

    # Create lists to store the data for each data source
    battery_amp_data = []
    solar_volt_data = []
    battery_volt_data = []

    # Define the MQTT client callbacks
    def on_connect(client, userdata, flags, rc):
        print("Connected to MQTT broker with result code "+str(rc))
        # Subscribe to the MQTT topics
        client.subscribe([(mqtt_topic_battery_amp, 0), (mqtt_topic_solar_volt, 0), (mqtt_topic_battery_volt, 0)])

    def on_message(client, userdata, msg):
        # Parse the MQTT message and update the corresponding data list
        data = json.loads(msg.payload)
        if msg.topic == mqtt_topic_battery_amp:
            battery_amp_data.append(data["value"])
        elif msg.topic == mqtt_topic_solar_volt:
            solar_volt_data.append(data["value"])
        elif msg.topic == mqtt_topic_battery_volt:
            battery_volt_data.append(data["value"])

    # Create the MQTT client and connect to the broker
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(mqtt_broker, mqtt_port, 60)

    # Create the battery amp graph and add it to the window
    ax1.set_ylim([0, 10])
    ax1.set_title("Battery Amp (A)")
    ax1.set_xlabel("Time")
    ax1.grid(True)
    line1, = ax1.plot([], [], colors["battery_amp"] + ".-")
    canvas1 = FigureCanvasTkAgg(fig, master=root)
    canvas1.draw()
    canvas1.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # Create the solar volt graph and add it to the window
    ax2.set_ylim([0, 20])
    ax2.set_title("Solar Volt (V)")
    ax2.set_xlabel("Time")
    ax2.grid(True)
    line2, = ax2.plot([], [], colors["solar_volt"] + ".-")
    canvas2 = FigureCanvasTkAgg(fig, master=root)
    canvas2.draw()
    canvas2.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # Create the battery volt graph and add it to the window
    ax3.set_ylim([0, 20])
    ax3.set_title("Battery Volt (V)")
    ax3.set_xlabel("Time")
    ax3.grid(True)
    line3, = ax3.plot([], [], colors["battery_volt"] + ".-")
    canvas3 = FigureCanvasTkAgg(fig, master=root)
    canvas3.draw()
    canvas3.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # Define the function to update the graphs
    def update():
        # Update the data for each graph
        line1.set_xdata(range(len(battery_amp_data)))
        line1.set_ydata(battery_amp_data)
        line2.set_xdata(range(len(solar_volt_data)))
        line2.set_ydata(solar_volt_data)
        line3.set_xdata(range(len(battery_volt_data)))
        line3.set_ydata(battery_volt_data)
        # Redraw the canvas
        canvas1.draw()
        canvas2.draw()
        canvas3.draw()
        # Schedule the next update
        root.after(update_interval, update)

    # Schedule the first update
    root.after(update_interval, update)

    # Start the tkinter main loop
    root.mainloop()
    
update_graphs()