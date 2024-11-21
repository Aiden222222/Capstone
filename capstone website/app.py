from flask import Flask, request, render_template, redirect, url_for, jsonify
from flask_mysqldb import MySQL
import paho.mqtt.client as mqtt
import threading
from datetime import datetime
import mysql.connector
import bcrypt
import os
import random
import json
import smtplib
from email.mime.text import MIMEText

app = Flask(__name__)
data_storage = {"temperature": None, # Stores the latest temperature received
                "container_filled": None,  # Initial temperature setpoint
                "setpoint": 6.0, #Initial temperature setpoint (default value)
                "daily_temperatures": {} # Stores temperatures for each day
}
user_email = None  # Store the email entered during signup

def get_user_email(box_id):
    cur = mysql.connection.cursor()
    cur.execute("SELECT email FROM users WHERE box_id = %s", (box_id,))
    result = cur.fetchone()
    cur.close()
    if result:
        return result[0]  # Return the email
    return None

# MQTT Settings
BROKER_IP = "test.mosquitto.org"
port = 1883
TOPIC_DATA = "/Medsafe/data"  # ESP32 sends temperature here
TOPIC_SETPOINT = "/Medsafe/setpoint"  # Server sends setpoint here

# Create global MQTT client
mqtt_client = mqtt.Client()

# Configure MySQL from environment variables
app.config['MYSQL_HOST'] = os.environ.get('MYSQL_HOST', 'mysql')
app.config['MYSQL_USER'] = os.environ.get('MYSQL_USER', 'root')
app.config['MYSQL_PASSWORD'] = os.environ.get('MYSQL_PASSWORD', 'root')
app.config['MYSQL_DB'] = os.environ.get('MYSQL_DB', 'Capstone')

# Initialize MySQL
mysql = MySQL(app)

# MQTT Callback to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    client.subscribe(TOPIC_DATA)  # Subscribe to the data topic
    print("MQTT client connected and subscribed.")

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
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message
        mqtt_client.connect(BROKER_IP, port, 60)
        mqtt_client.loop_forever()

    threading.Thread(target=run_mqtt).start()

# Function to send an email
def send_email(body, recipient):
    try:
        sender_email = "capstone093@gmail.com"  # Updated sender email
        sender_password = "Medsafe2024"  # Updated sender password
        smtp_server = "smtp.gmail.com"
        smtp_port = 587

        # Create email
        msg = MIMEText(body)
        msg["Subject"] = "MedSafe Package Received"  # Fixed subject line
        msg["From"] = sender_email
        msg["To"] = recipient

        # Send email
        with smtplib.SMTP(smtp_server, smtp_port) as server:
            server.starttls()
            server.login(sender_email, sender_password)
            server.send_message(msg)

        print(f"Email sent to {recipient}")
    except Exception as e:
        print(f"Failed to send email: {e}")




# Flask endpoint to retrieve the latest message
@app.route('/latest-data', methods=['GET'])
def get_latest_data():
    box_id = request.form.get('box_id')
    user_email = get_user_email(box_id)
    if jsonify(data_storage):
        if user_email:  # Check if email is set
            send_email(
                body="A package is in the box.",  # Body of the email
                recipient=user_email  # Recipient's email
            )
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
    mqtt_client.publish(TOPIC_SETPOINT, str(new_setpoint))

    print(f"Published new setpoint: {new_setpoint}")

    mqtt_client.disconnect()

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
        # Query the database to validate user credentials
        cur = mysql.connection.cursor()
        query = "SELECT password FROM users WHERE box_id = %s"
        cur.execute(query, (box_id,))
        result = cur.fetchone()  # Fetch one row
        stored_password = result[0]
        if bcrypt.checkpw(password.encode('utf-8'), stored_password.encode('utf-8')):
            # Successful login, redirect to display page
            cur.close()
            return redirect(url_for('display', box_id=box_id))
        else:
            # Invalid password
            cur.close()
            return "Invalid credentials", 401
        result = cur.fetchone()  # Fetch one row
        return redirect(url_for('display', box_id=box_id))
    return render_template('login.html')

# Signup route
@app.route('/signup', methods=['GET', 'POST'])
def signup():
    if request.method == 'POST':
        box_id = request.form['boxId']
        password = request.form['password']
        user_email = request.form['Email']  # Capture email from form

        hashed_password = bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt())
        cur = mysql.connection.cursor()
        cur.execute("INSERT INTO users (box_id, password, email) VALUES (%s, %s, %s)", (box_id, hashed_password, user_email))
        mysql.connection.commit()
        cur.close()
        return redirect(url_for('display', box_id=box_id))
    return render_template('signup.html')

# Display route
@app.route('/display')
def display():
    box_id = request.args.get('box_id')
    return render_template('display.html', box_id=box_id)

@app.route('/check-temperature', methods=['POST'])
def check_temperature():
    try:
        # Log incoming request
        print(f"Request data: {request.form}")

        # Retrieve form data
        day = request.form.get('day')
        temperature = request.form.get('temperature')

        # Validate form inputs
        if not day or not temperature:
            print("Day or temperature missing.")
            return jsonify(success=False, error="Day or temperature missing."), 400

        # Get the current day of the week
        current_day = datetime.now().strftime('%A')

        # Check if the day matches the current day ***USE TEMPERATURE FROM THIS IF TO SEND TO ESP32
        if day == current_day:
            print(f"Temperature for {day}: {temperature}")

            # Update the temperature for the day in the data storage
            if "daily_temperatures" not in data_storage:
                data_storage["daily_temperatures"] = {}

            data_storage["daily_temperatures"][day] = temperature

            # Publish the new setpoint to the ESP32 (check if connected before publishing)
            if mqtt_client.is_connected():
                mqtt_client.publish(TOPIC_SETPOINT, str(temperature))
                print(f"Published new setpoint: {temperature}")
                return jsonify({"message": "Setpoint sent to MQTT broker", "new_setpoint": temperature})
            else:
                print("MQTT client is not connected.")
                return jsonify(success=False, error="MQTT client not connected."), 500
        else:
            print(f"Day mismatch: Today is {current_day}, not {day}.")
            return jsonify(success=False, error=f"Today is {current_day}, not {day}.")
    except KeyError as e:
        print(f"KeyError: {e}")
        return jsonify(success=False, error=f"Invalid day key: {e}"), 400
    except Exception as e:
        print(f"Unexpected error: {e}")
        return jsonify(success=False, error=f"An unexpected error occurred: {e}"), 500

if __name__ == '__main__':
    # Start the MQTT client thread
    start_mqtt_client()

    app.run(host='0.0.0.0', port=5000)  # Start Flask app





