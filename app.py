from flask import Flask, request, render_template, redirect, url_for, jsonify
import paho.mqtt.client as mqtt
import threading

app = Flask(__name__)
data_storage = {"latest_data": "No data received yet."}  # Storage for latest message

# MQTT Settings
BROKER_IP = "test.mosquitto.org" 
port = 1883
TOPIC = "/Capstone_Medical_Device/data"

# MQTT Callback to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    client.subscribe(TOPIC)

# MQTT Callback to handle received messages
def on_message(client, userdata, msg):
    # global data_storage
    data_storage["latest_data"] = msg.payload.decode()
    print(f"Received message: {data_storage['latest_data']}")

# Function to start the MQTT client
def start_mqtt_client():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER_IP, port, 60)
    client.loop_forever()

# Flask endpoint to retrieve the latest message
@app.route('/latest-data', methods=['GET'])
def get_latest_data():
    return jsonify(data_storage)

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
    mqtt_thread = threading.Thread(target=start_mqtt_client)
    mqtt_thread.start()  # Start MQTT in a separate thread

    app.run(host='0.0.0.0', port=5000)  # Start Flask app





