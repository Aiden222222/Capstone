This project is part of a capstone aimed at developing a secure, temperature-controlled dropbox for the safe delivery and temporary storage of temperature-sensitive medications. Many people rely on delivered medications that must be kept within specific temperature ranges to remain effective. Our solution provides a home-installed box that keeps these medications secure and actively monitors temperature, with the ability to regulate it as needed.

Key Features
Temperature Monitoring and Control: Actively monitors the internal temperature of the box and adjusts it as needed based on incoming temperature data.
Secure Delivery Notifications: Users can receive alerts when a package arrives, and the box locks itself to prevent unauthorized access.
Remote Configuration: Backend can update target temperature settings and notify the box of scheduled deliveries.


In order to use this first make sure you install
paho-mqtt using pip install paho-mqtt. Also replace
the placeholders in the env_config.h file for wifi_ssid and password, with the correct values on 2.4GHz.
