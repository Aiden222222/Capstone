from flask import Flask, request, render_template, redirect, url_for, jsonify
import paho.mqtt.client as mqtt
import threading
import time
import random

app = Flask(__name__)
data_storage = {"temperature": "No data received yet.", # Stores the latest temperature received
                "setpoint": 10.0  # Initial temperature setpoint
}  

# MQTT Settings
BROKER_IP = "test.mosquitto.org" 
port = 1883
TOPIC_TEMPERATURE = "/Medsafe/temperature"  #ESP32 sends temperature here
TOPIC_SETPOINT = "/Medsafe/setpoint"    # Server sends setpoint here

# MQTT Callback to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    client.subscribe(TOPIC_TEMPERATURE)

# MQTT Callback to handle received messages
def on_message(client, userdata, msg):
    if msg.topic == TOPIC_TEMPERATURE:
        # Update the temperature in data_storage
        data_storage["temperature"] = msg.payload.decode()
        print(f"Received temperature: {data_storage['temperature']}")

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

# Flask endpoint to send setpoint to MQTT broker
@app.route('/send-setpoint', methods=['POST'])
def send_setpoint():
    time.sleep(10)
    new_setpoint = random.uniform(5.0, 20.0)
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





