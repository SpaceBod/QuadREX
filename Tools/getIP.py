import requests
import socket
import time
from datetime import datetime
import subprocess

# ThingSpeak API settings
API_KEY = ''
CHANNEL_ID = ''
UPDATE_URL = f'https://api.thingspeak.com/update.json'

def get_ip_address():
    try:
        hostname = socket.gethostname()
        ip_address = socket.gethostbyname(hostname + ".local")
        if ip_address.startswith("127."):  # Filter out loopback address
            return None
        return ip_address
    except socket.error:
        return None

def post_to_thingspeak(ip):
    payload = {
        'api_key': API_KEY,
        'field1': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
        'field2': ip
    }
    try:
        response = requests.post(UPDATE_URL, data=payload)
        if response.status_code == 200:
            subprocess.run(["espeak", "WiFi connected"])
    except requests.exceptions.RequestException as e:
        print('Failed to connect to ThingSpeak. Retrying...', str(e))
        time.sleep(10)  # Shorter wait before retrying if there's a connection issue
        post_to_thingspeak(ip)

def main():
    last_ip = None
    check_interval = 10  # Start with 10 seconds interval assuming no connection initially

    while True:
        try:
            current_ip = get_ip_address()
            if current_ip:
                if current_ip != last_ip:
                    print(f'IP Address changed to {current_ip}')
                    post_to_thingspeak(current_ip)
                    last_ip = current_ip
                    check_interval = 30  # Extend interval after a successful update
                else:
                    print('No IP Address change detected.')
            else:
                print('No valid external IP address found. Possibly no internet connection.')
                check_interval = 10  # Reduce interval to check again soon

            time.sleep(check_interval)  # Dynamic adjustment of the check interval

        except Exception as e:
            print(f'Error: {e}')
            check_interval = 10  # If an error occurs, assume potential connection issues and check sooner
            time.sleep(check_interval)

if __name__ == '__main__':
    main()
