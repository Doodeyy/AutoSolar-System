import tkinter as tk
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

# Define MQTT settings
broker_address = "localhost"
broker_port = 1883
battery_amp_topic = "sensors/battery_amp"
solar_volt_topic = "sensors/solar_volt"
battery_volt_topic = "sensors/battery_volt"

# Define graph settings
graph_width = 8
graph_height = 4
max_data_points = 50
update_interval = 1000  # in milliseconds

# Create a deque to store data points for each graph
battery_amp_data = deque(maxlen=max_data_points)
solar_volt_data = deque(maxlen=max_data_points)
battery_volt_data = deque(maxlen=max_data_points)

# Define the function to update the graphs
def update_graphs():
    # Connect to the MQTT broker
    client = mqtt.Client()
    client.connect(broker_address, broker_port)

    # Subscribe to the MQTT topics
    client.subscribe(battery_amp_topic)
    client.subscribe(solar_volt_topic)
    client.subscribe(battery_volt_topic)

    # Define the callback function to receive MQTT messages
    def on_message(client, userdata, message):
        # Decode the message payload as a float
        value = float(message.payload.decode("utf-8"))

        # Add the value to the deque for the corresponding graph
        if message.topic == battery_amp_topic:
            battery_amp_data.append(value)
        elif message.topic == solar_volt_topic:
            solar_volt_data.append(value)
        elif message.topic == battery_volt_topic:
            battery_volt_data.append(value)

    # Set the MQTT message callback function and start the loop
    client.on_message = on_message
    client.loop_start()

    # Create the battery amp graph
    fig1 = plt.Figure(figsize=(graph_width, graph_height))
    ax1 = fig1.add_subplot(111)
    ax1.set_ylim([0, 10])
    ax1.plot(battery_amp_data, "r.-")
    ax1.set_title("Battery Amp (A)")
    ax1.set_xlabel("Time")

    # Create the solar volt graph
    fig2 = plt.Figure(figsize=(graph_width, graph_height))
    ax2 = fig2.add_subplot(111)
    ax2.set_ylim([0, 20])
    ax2.plot(solar_volt_data, "g.-")
    ax2.set_title("Solar Volt (V)")
    ax2.set_xlabel("Time")

    # Create the battery volt graph
    fig3 = plt.Figure(figsize=(graph_width, graph_height))
    ax3 = fig3.add_subplot(111)
    ax3.set_ylim([10, 14])
    ax3.plot(battery_volt_data, "b.-")
    ax3.set_title("Battery Volt (V)")
    ax3.set_xlabel("Time")

    # Create the tkinter window
    root = tk.Tk()
    root.title("Solar Panel Monitoring")
    root.geometry("800x600")
    root.attributes("-fullscreen" , False)

    # Create the battery amp graph canvas and add it to the window
    canvas1 = FigureCanvasTkAgg(fig1, master=root)
    canvas1.draw()
    canvas1.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # Create the solar volt graph canvas and add it to the window
    canvas2 = FigureCanvasTkAgg(fig2 , master=root)
    canvas2.draw()
    canvas2.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
    
    # Create the battery volt graph canvas and add it to the window
    canvas3 = FigureCanvasTkAgg(fig3, master=root)
    canvas3.draw()
    canvas3.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # Define the function to update the graphs periodically
    def update_graphs_periodic():
        # Redraw the graphs with the updated data
        ax1.clear()
        ax1.set_ylim([0, 10])
        ax1.plot(battery_amp_data, "r.-")
        ax1.set_title("Battery Amp (A)")
        ax1.set_xlabel("Time")

        ax2.clear()
        ax2.set_ylim([0, 20])
        ax2.plot(solar_volt_data, "g.-")
        ax2.set_title("Solar Volt (V)")
        ax2.set_xlabel("Time")

        ax3.clear()
        ax3.set_ylim([10, 14])
        ax3.plot(battery_volt_data, "b.-")
        ax3.set_title("Battery Volt (V)")
        ax3.set_xlabel("Time")
        
        # Schedule the next update after the specified interval
        root.after(update_interval, update_graphs_periodic)
    
    
    #def switchMode():
     #   if root.attributes("-fullscreen" , False):
      #      root.attributes = ("-fullscreen", True)
            
       # elif root.attributes("-fullscreen" , True):
        #    root.attributes = ("-fullscreen", False)
            
    #fsMode = tk.Button(root, text="Screen Mode", command=switchMode)
   # fsMode.place(x =  0, y = 0)
   
    def toggle_fullscreen():
    # get the current fullscreen state
        state = root.attributes("-fullscreen")

    # toggle the state
        root.attributes("-fullscreen", not state)



    button = tk.Button(root, text="Toggle Fullscreen", command=toggle_fullscreen)
    button.place(x =  0, y = 0)

    
    
# Schedule the initial update of the graphs
    update_graphs_periodic()
    
    
    
# Start the tkinter event loop
    root.mainloop()



update_graphs()