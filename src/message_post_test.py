import requests

def send_text_message_to_server(message):
        url = "http://192.168.4:8000/api/text-message"
        request_obj = {'message' : message}
        response = requests.post(url, data = request_obj)
        return response   


response = send_text_message_to_server("Test from text message post test.")
print(response.status_code)