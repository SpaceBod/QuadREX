import requests


def fetch_from_thingspeak(channel_id, api_key):
    url = f"https://api.thingspeak.com/channels/{channel_id}/feeds.json?api_key={api_key}&results=10"
    response = requests.get(url)

    if response.status_code == 200:
        data = response.json()
        feeds = data["feeds"]
        print("Fetched Data:")
        for feed in feeds:
            print(f"Time: {feed['field1']}")
            print(f"Name: {feed['field2']}")
            print(f"Age: {feed['field3']}")
            print(f"Type: {feed['field4']}")
            print(f"Details: {feed['field5']}")
            print(f"Severity: {feed['field6']}")
            print("-" * 40)
    else:
        print("Failed to fetch data from ThingSpeak:", response.text)


if __name__ == "__main__":
    # ThingSpeak channel details
    channel_id = "2561068"
    read_api_key = "H8LE7O9V8GLI0LLV"

    # Fetch and display data
    fetch_from_thingspeak(channel_id, read_api_key)
