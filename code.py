import time, gc, os
import adafruit_dotstar
import board
import feathers2
import json
import wifi
import socketpool
import adafruit_scd4x
import adafruit_bme680
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import microcontroller


# Make sure the 2nd LDO is turned on
feathers2.enable_LDO2(True)

# Create a DotStar instance
pixel = adafruit_dotstar.DotStar(board.APA102_SCK, board.APA102_MOSI, 1, brightness=0.5, auto_write=True)

# Turn off the internal blue LED
feathers2.led_set(False)

# Show available memory
print("Memory Info - gc.mem_free()")
print("---------------------------")
print("{} Bytes\n".format(gc.mem_free()))

flash = os.statvfs('/')
flash_size = flash[0] * flash[2]
flash_free = flash[0] * flash[3]
# Show flash size
print("Flash - os.statvfs('/')")
print("---------------------------")
print("Size: {} Bytes\nFree: {} Bytes\n".format(flash_size, flash_free))

# Use a separate file for secrets
try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    pixel[0] = (0,0,255) # Set pixel to blue
    raise
# MQTT Topic
# Use this topic if you'd like to connect to a standard MQTT broker
mqtt_topic = secrets["mqtt_topic"]


def connect(mqtt_client, userdata, flags, rc):
    # This function will be called when the mqtt_client is connected
    # successfully to the broker.
    print("Connected to MQTT Broker!")
    print("Flags: {0}\n RC: {1}".format(flags, rc))


def disconnect(mqtt_client, userdata, rc):
    # This method is called when the mqtt_client disconnects
    # from the broker.
    print("Disconnected from MQTT Broker!")


def subscribe(mqtt_client, userdata, topic, granted_qos):
    # This method is called when the mqtt_client subscribes to a new feed.
    print("Subscribed to {0} with QOS level {1}".format(topic, granted_qos))


def unsubscribe(mqtt_client, userdata, topic, pid):
    # This method is called when the mqtt_client unsubscribes from a feed.
    print("Unsubscribed from {0} with PID {1}".format(topic, pid))


def publish(mqtt_client, userdata, topic, pid):
    # This method is called when the mqtt_client publishes data to a feed.
    print("Published to {0} with PID {1}".format(topic, pid))


def message(client, topic, message):
    # Method called when a client's subscribed feed has a new value.
    print("New message on topic {0}: {1}".format(topic, message))


# Create a socket pool
pool = socketpool.SocketPool(wifi.radio)

# Set up a MiniMQTT Client
mqtt_client = MQTT.MQTT(
    broker=secrets["mqtt_broker"],
    port=secrets["mqtt_port"],
    socket_pool=pool
)

# Connect callback handlers to mqtt_client
mqtt_client.on_connect = connect
mqtt_client.on_disconnect = disconnect
mqtt_client.on_subscribe = subscribe
mqtt_client.on_unsubscribe = unsubscribe
mqtt_client.on_publish = publish
mqtt_client.on_message = message


# Initialize the sensors on the STEMMA connector
i2c = board.STEMMA_I2C()
scd4x = adafruit_scd4x.SCD4X(i2c)
scd4x.self_calibration_enabled = False
bme = adafruit_bme680.Adafruit_BME680_I2C(i2c, address=0x77)

pixel.brightness = 0.01
# pixel[0] = (0,255,0)
# Initial burn in the BME688 sensor for 15 min
mono_time = time.monotonic()
while time.monotonic() < (mono_time + 900):
    print('Gas: {} ohms'.format(bme.gas))
    #time.sleep(0.5)

# Make sure BME burnin is only run once while waiting for the SCD to be ready
bme_burnin = False
scd_started = False
while True:
    pixel[0] = (0,0,0)
    mono_time = time.monotonic()
    if not bme_burnin:
        while time.monotonic() < (mono_time + 600):
            print('Gas: {} ohms'.format(bme.gas))
            bme_burnin = True
    print('Final Gas: {} ohms'.format(bme.gas))
    last_bme_gas = bme.gas
    if not scd_started:
        scd4x.start_periodic_measurement()
        scd_started = True
    if scd4x.data_ready:
        print("++++++++++")
        print("CO2: %d ppm" % scd4x.CO2)
        print("TemperatureSCD: %0.1f *C" % scd4x.temperature)
        print("HumiditySCD: %0.1f %%" % scd4x.relative_humidity)
        print()
        print('Temperature: {} degrees C'.format(bme.temperature))
        print('Gas: {} ohms'.format(bme.gas))
        print('Humidity: {}%'.format(bme.humidity))
        print('Pressure: {}hPa'.format(bme.pressure))
        
        sensor_data = {
            "tags" : {
                "deviceType" : "AirSensor",
                "deviceLocation" : "office-1",
                "sensorNumber" : 2
            },
            "fields" : {
                "temp" : scd4x.temperature,
                "hum" :  scd4x.relative_humidity,
                "co2" : scd4x.CO2,
                "baro" : bme.pressure,
                "gas" : last_bme_gas

            }
        }
        
        # Connect to wifi
        print("Connecting to %s" % secrets["wifi_ssid"])
        connection_attempt = 0
        connection_retry = True
        
        while connection_attempt < 5 and connection_retry:
            wifi.radio.enabled = True
            try:
                wifi.radio.connect(secrets["wifi_ssid"], secrets["wifi_pass"])
                print("Connected to %s! with IP %s" % (secrets["wifi_ssid"], wifi.radio.ipv4_address ))
                time.sleep(0.5)
                mqtt_client.connect()
                mqtt_client.publish(mqtt_topic, json.dumps(sensor_data))
                mqtt_client.disconnect()
                wifi.radio.enabled = False
                connection_retry = False
                #break
            except Exception as error_message:
                pixel[0] = (2,0,0) # Set Neopixel to red
                print(f'Connection failed, this was attempt {connection_attempt}')
                print(error_message)
                connection_attempt += 1
                if connection_attempt >= 3 and connection_attempt < 5:
                    wifi.radio.enabled = False
                    time.sleep(600) #  Wait 10 mins
                if connection_attempt == 5:
                    microcontroller.reset()

        # Reset BME burnin
        bme_burnin = False

        scd4x.stop_periodic_measurement()
        scd_started = False

        time.sleep(1)