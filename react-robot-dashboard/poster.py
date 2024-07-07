import json
import requests
import time
import sys

def log(message):
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {message}")

def post_payload(ip_address, port, payload):
    url = f"http://{ip_address}:{port}/publishData"
    headers = {'Content-Type': 'application/json'}
    while True:
        try:
            payload["usonic"] = payload["usonic"] + 1
            response = requests.post(url, data=json.dumps(payload), headers=headers, timeout=0.3)
            if response.status_code == 200:
                log("Payload posted successfully")
            else:
                log(f"___________________________________________Failed to post payload. Status code: {response.status_code}")
        except requests.exceptions.RequestException as e:
            log(f"_____________________________________________An error occurred: {e}")
        time.sleep(0.05)  # Sleep for 100ms

# Example usage
ip_address = "192.168.137.141"
port = 5000
# Get color from command line argument
color = sys.argv[1]

payload = {"color": color, "servo": 90, "motora": 0, "motorb": 0, "cpsa": 69, "cpsb": 699, "cnta": 111, "cntb": 222, "usonic": 22, "hsv": {"h": 0, "s": 0, "v": 0}}

post_payload(ip_address, port, payload)