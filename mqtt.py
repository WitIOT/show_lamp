import time
from machine import Pin, DAC, ADC
from time import sleep
import machine
import utime
import network
from umqtt.simple import MQTTClient

# Wi-Fi configuration
# WIFI_SSID = "ck_show_iot"
# WIFI_PASSWORD = "ck@12345678"

WIFI_SSID = "CKTECH_2.4GHz"
WIFI_PASSWORD = "0817830056"

# MQTT configuration
# MQTT_BROKER = "183.88.229.11"

MQTT_BROKER = "192.168.1.14"

MQTT_PORT = 1885
MQTT_USERNAME = "admin1"
MQTT_PASSWORD = "12345678"
MQTT_LED_TOPIC = "led_st"


# นับเวลาตั้งแต่เริ่มต้น
start_time = time.time()
diml = 0
d = 0
Dim = DAC(Pin(26))

pv = ADC(Pin(34))
pv.atten(ADC.ATTN_11DB)
vbat = ADC(Pin(35))
vbat.atten(ADC.ATTN_11DB)
sss = ADC(Pin(32))
sss.atten(ADC.ATTN_11DB)

led = Pin(14, Pin.OUT)
led_va = 0
def dim(t):
    di = int(t)
    Dim.write(di)
    
def mqtt_callback(topic, msg):
    global led
    global diml
    global led_va
    global d

    if topic == b'led_slider':
        d = int(msg)
        # Add this line to set diml to d when led_va is 1
        if led_va == 1:
            diml = d

    if topic == b'led_st':
        value = int(msg)
        led.value(value)
        led_va = value
        if led_va == 0:
            diml = 0
        if led_va == 1:
            diml = d
        
def connect_to_wifi():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print("Connecting to Wi-Fi...")
        sta_if.active(True)
        sta_if.connect(WIFI_SSID, WIFI_PASSWORD)
        while not sta_if.isconnected():
            pass
    print("Wi-Fi connected. IP:", sta_if.ifconfig()[0])

def connect_to_mqtt():
    client = MQTTClient("esp32", MQTT_BROKER, port=MQTT_PORT, user=MQTT_USERNAME, password=MQTT_PASSWORD)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(b'led_st')
    client.subscribe(b'led_slider')
    print("Connected to MQTT broker")
    return client

connect_to_wifi()
mqtt_client = connect_to_mqtt()

while True:
    try:
        mqtt_client.check_msg()
        
        pvr = pv.read()
        vbatr = vbat.read()
        print('pv:', pvr)
        print('batt:', vbatr)
        
        print('Dim:', diml)
        print('led pin:', led_va)
        print(' ')
        dim(diml)
#         time.sleep(1)
    except KeyboardInterrupt:
        print("Disconnecting from MQTT broker and Wi-Fi...")
        mqtt_client.disconnect()
        machine.reset()
