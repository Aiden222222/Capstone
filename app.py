from flask import Flask, request, render_template, redirect, url_for, jsonify
import paho.mqtt.client as mqtt
import threading
import json

app = Flask(__name__)
data_storage = {"temperature": None, # Stores the latest temperature received
                "container_filled": None,  # Initial temperature setpoint
                "setpoint": 6.0 #Initial temperature setpoint (default value)
}  

# MQTT Settings
BROKER_IP = "test.mosquitto.org" 
port = 1883
TOPIC_DATA = "/Medsafe/data"  #ESP32 sends temperature here
TOPIC_SETPOINT = "/Medsafe/setpoint"    # Server sends setpoint here

# MQTT Callback to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    client.subscribe(TOPIC_DATA)

# MQTT Callback to handle received messages
def on_message(client, userdata, msg):
    if msg.topic == TOPIC_DATA:
        try:
            # Parse the JSON payload
            payload = json.loads(msg.payload.decode())
            data_storage["temperature"] = payload.get("temperature", None)
            data_storage["container_filled"] = payload.get("container_filled", None)
            
            # Print the received data
            print(f"Received data - Temperature: {data_storage['temperature']}°C, "
                  f"Container Filled: {data_storage['container_filled']}")
        except json.JSONDecodeError:
            print("Failed to decode JSON payload")

# Function to start the MQTT client
def start_mqtt_client():
    def run_mqtt():
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(BROKER_IP, port, 60)
        client.loop_forever()
    threading.Thread(target=run_mqtt).start()


# Flask endpoint to retrieve the latest message
@app.route('/latest-data', methods=['GET'])
def get_latest_data():
    return jsonify(data_storage)

# Flask endpoint to retrieve the current setpoint
@app.route('/get-setpoint', methods=['GET'])
def get_setpoint():
    return jsonify({"setpoint": data_storage["setpoint"]})

# Flask endpoint to send setpoint to MQTT broker
@app.route('/send-setpoint', methods=['POST'])
def send_setpoint():
    new_setpoint = request.json.get("setpoint", data_storage["setpoint"])
    new_setpoint = round(new_setpoint, 2)

    # Update the setpoint in the data storage
    data_storage["setpoint"] = new_setpoint

    # Publish the new setpoint to the ESP32 via MQTT
    client = mqtt.Client()
    client.connect(BROKER_IP, port, 60)
    client.publish(TOPIC_SETPOINT, str(new_setpoint))
    print(f"Published new setpoint: {new_setpoint}")
    client.disconnect()
    
    return jsonify({"message": "Setpoint sent to MQTT broker", "new_setpoint": new_setpoint})
    

# Homepage
@app.route('/')
def index():
    return render_template('index.html')

# Login route
@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        box_id = request.form['boxId']
        password = request.form['password']
        return redirect(url_for('display', box_id=box_id))
    return render_template('login.html')

# Signup route
@app.route('/signup', methods=['GET', 'POST'])
def signup():
    if request.method == 'POST':
        box_id = request.form['boxId']
        password = request.form['password']
        return redirect(url_for('display', box_id=box_id))
    return render_template('signup.html')

# Display route
@app.route('/display')
def display():
    box_id = request.args.get('box_id')
    return render_template('display.html', box_id=box_id)

if __name__ == '__main__':
    # Start the MQTT client thread
    start_mqtt_client()
    app.run(host='0.0.0.0', port=5000)  # Start Flask app





