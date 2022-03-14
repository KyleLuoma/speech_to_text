import requests

def send_text_message_to_server(message):
        url = "http://192.168.1.4:8000/api/text-message"
        request_obj = {'text' : message, "type" : "message"}
        response = requests.post(url, data = request_obj)
        return response   


#response = send_text_message_to_server("Test from text message post test.")

url = "http://192.168.1.4:8000/api/text-message"
song_name = "test song name"
request_obj = {'text' : song_name, "type" : "song"}
try:
        response = requests.post(url, data = request_obj)
        #rospy.loginfo("Submited song to server and received response " + response.status_code)
        self.jam_pub.publish(True)
except:
        pass
        #rospy.loginfo("Ran into trouble trying to communicate with the song server")


print(response.status_code)