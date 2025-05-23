# Import standard python modules.
import sys
import time
import serial

# This example uses the MQTTClient instead of the REST client
from Adafruit_IO import MQTTClient
from Adafruit_IO import Client, Feed

# holds the count for the feed
run_count = 0

# Set to your Adafruit IO username and key. 
# Remember, your key is a secret,
# so make sure not to publish it when you publish this code!
ADAFRUIT_IO_USERNAME = "pato99"
ADAFRUIT_IO_KEY = ""

# Set to the ID of the feed to subscribe to for updates.
FEED_servo1_RX = 'proyecto-silla.servo1-rx'
FEED_servo1_TX = 'proyecto-silla.servo1-tx'
FEED_servo2_RX = 'proyecto-silla.servo2-rx'
FEED_servo2_TX = 'proyecto-silla.servo2-tx'
FEED_servo3_RX = 'proyecto-silla.servo3-rx'
FEED_servo3_TX = 'proyecto-silla.servo3-tx'
FEED_servo4_RX = 'proyecto-silla.servo4-rx'
FEED_servo4_TX = 'proyecto-silla.servo4-tx'

FEED_modos = 'proyecto-silla.modos'
FEED_poisson = 'proyecto-silla.poisson'

# Define "callback" functions which will be called when certain events 
# happen (connected, disconnected, message arrived).
def connected(client):
    """Connected function will be called when the client is connected to
    Adafruit IO.This is a good place to subscribe to feed changes. The client
    parameter passed to this function is the Adafruit IO MQTT client so you
    can make calls against it easily.
    """
    # Subscribe to changes on a feed named Counter.
    print('Subscribing to Feeds... ')
    client.subscribe(FEED_servo1_RX)
    client.subscribe(FEED_servo2_RX)
    client.subscribe(FEED_servo3_RX)
    client.subscribe(FEED_servo4_RX)
    client.subscribe(FEED_modos)
    client.subscribe(FEED_poisson)
    print('Waiting for feed data...')

def disconnected(client):
    """Disconnected function will be called when the client disconnects."""
    sys.exit(1)

def message(client, feed_id, payload):
    """Message function will be called when a subscribed feed has a new value.
    The feed_id parameter identifies the feed, and the payload parameter has
    the new value.
    """
    
    print('Feed {0} received new value: {1}'.format(feed_id, payload))
    # Publish or "send" message to corresponding feed  
    if feed_id == FEED_servo1_RX:
        prefijo = '1'
    elif feed_id == FEED_servo2_RX:
        prefijo = '2'
    elif feed_id == FEED_servo3_RX:
        prefijo = '3'
    elif feed_id == FEED_servo4_RX:
        prefijo = '4'
    elif feed_id == FEED_modos:
        prefijo = 'M'
    elif feed_id == FEED_poisson:
        prefijo = 'P'
    else:
        return  

    miarduino.write(bytes(f"{prefijo}{payload},", 'utf-8'))
    print(f'Sending data back: {payload}')


miarduino = serial.Serial(port = 'COM6', baudrate=9600, timeout=0.1)

# Create an MQTT client instance.
client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Setup the callback functions defined above.
client.on_connect = connected
client.on_disconnect = disconnected
client.on_message = message

# Connect to the Adafruit IO server.
client.connect()

# The first option is to run a thread in the background so you can continue
# doing things in your program.
client.loop_background()

mensaje = 1

while True:
    """ 
    # Uncomment the next 3 lines if you want to constantly send data
    # Adafruit IO is rate-limited for publishing
    # so we'll need a delay for calls to aio.send_data()
    run_count += 1
    print('sending count: ', run_count)
    client.publish(FEED_ID_Send, run_count)
    """
    if (mensaje == 1):
        print('Running "main loop" ')
        mensaje = 0

    if miarduino.in_waiting > 0:
        # Leer los valores enviados por el Arduino
        data = miarduino.readline().decode('utf-8').strip()  # Lee la línea y elimina saltos de línea
        print(f'Datos recibidos del Arduino: {data}')
        
        value = 0

        if data.startswith('1'):
            value = data[1:5]
            print(f"Valor del servo 1 es: {value}\n")
            client.publish(FEED_servo1_TX, value)
        elif data.startswith('2'):
            value = data[1:5]
            print(f"Valor del servo 2: {value}\n")
            client.publish(FEED_servo2_TX, value)
        elif data.startswith('3'):
            value = data[1:5]
            print(f"Valor del servo 3: {value}\n")
            client.publish(FEED_servo3_TX, value)
        elif data.startswith('4'):
            value = data[1:5]
            print(f"Valor del servo 4: {value}\n")
            client.publish(FEED_servo4_TX, value)

        print(f"Publicando en Adafruit: '{value}'")