import time
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
from pydub import AudioSegment, silence
import openai
from playsound import playsound
import tempfile
import os
import matplotlib.pyplot as plt
import pyaudio
import rclpy
from rclpy.node import Node

class OpenAIChatbot(Node):

    def __init__(self):
        super().__init__('chatbot')
        openai.api_key= os.getenv("OPENAI_API_KEY") #get api key as enviormental variable

        # speaker options are alloy, echo, fable, onyx, nova, and shimmer
        self.speaker = "alloy"  
        self.goal_location = 'unknown'

        self.guidebot_personality = '''
            You are a guide robot, only take english input and only respond in english, 
            you can guide people around the room '120' and bring them to 1 of 8 locations:
            120a, 120b, Door 1, Door 2, Drake's Desk, Emily's Desk, Percy's Desk, Demo Table. 
        ''' 

    def rms(self, audio_segment):
        return audio_segment.rms

    def measure_background_noise(self, duration=3, samplerate=44100):
        print("Calibrating for background noise...")
        audio_chunk = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
        sd.wait()

        audio_segment = AudioSegment(
            audio_chunk.tobytes(),
            frame_rate=samplerate,
            sample_width=audio_chunk.dtype.itemsize,
            channels=1
        )

        # rms_value = rms(audio_segment)
        # print(f"Background noise RMS value: {rms_value}")
        # return rms_value

        dBFS_value = audio_segment.dBFS
        print(f"Background noise dBFS value: {dBFS_value}")
        return dBFS_value

    def plot_audio(self, audio_segment):
        samples = np.array(audio_segment.get_array_of_samples())
        plt.figure(figsize=(10,4))
        plt.plot(samples)
        plt.title("Audio Plot")
        plt.xlabel("Sample")
        plt.ylabel("Amplitude")
        plt.show()

    def detect_audio_silence(self, audio_segment : AudioSegment, pause_duration, threshold):
        min_silence_len = 1000*pause_duration
    
        samples = np.array(audio_segment.get_array_of_samples())
        rms_value = np.sqrt(np.mean(np.square(samples)))

        # convertnig rms to dbfs
        rms_dBFS = 20 * np.log10(rms_value / 32768.0)
        print("RMS Value: ", rms_value)
        print("RMS to dBFS Value: ", rms_dBFS)
        return rms_dBFS < threshold

    def record_audio(self, threshold=-40.0, chunk_duration=2, pause_duration=2, samplerate=44100): # chunk duration was 1
        print('Recording...')
        audio_chunks = []
        pause_counter = 0

        while True:
            # record a chunk of audio
            audio_chunk = sd.rec(int(chunk_duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
            # wait until audio is done recording
            sd.wait() 

            # pause detection for audio to end
            audio_segment = AudioSegment(
                audio_chunk.tobytes(),
                frame_rate=samplerate,
                sample_width=audio_chunk.dtype.itemsize,
                channels=1
            )

            # testing audio
            # plot_audio(audio_segment)

            audio_chunks.append(audio_segment)
            print("No pause detected. Adding audio segment...")

            # detects any silence in the audio segment
            silence_detected = self.detect_audio_silence(audio_segment, pause_duration, threshold)
            # print(f"Silence detected: {silence_detected}")
            if silence_detected:
                pause_counter += 1
            else:
                pause_counter = 0
            
            # stops recording if pause is detected
            if pause_counter > 0:
                print("Pause detected. Stopping detection...")
                break

        # combines the audio chunks until the silence threshold was met
        combined_audio = sum(audio_chunks)

        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_audio_file:
            combined_audio.export(temp_audio_file.name, format="wav")
            print("Recording complete.")
            return temp_audio_file.name
        
    def speech_to_text(self, audio_file):
        with open(audio_file, "rb") as file:
            response = openai.audio.transcriptions.create(
                model='whisper-1',
                file=file
            )

        response_text = response.text
        print("Query: ", response_text)
        return response_text

    def generate_response(self, query, personality):
        
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": personality},
                {"role": "user", "content": query},
            ],
        )
        response_data = response.choices[0].message.content
        print("AI Response: ", response_data)

        return response_data.replace("'", "")

    def text_to_speech(self, text):
        player_stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=24000, output=True) 

        start_time = time.time() 

        with openai.audio.speech.with_streaming_response.create( 
            model="tts-1", 
            voice=self.speaker, 
            response_format="pcm",  # similar to WAV, but without a header chunk at the start. 
            input=text, 
        ) as response: 
            print(f"Time to first byte: {int((time.time() - start_time) * 1000)}ms") 
            for chunk in response.iter_bytes(chunk_size=1024): 
                player_stream.write(chunk) 

        print(f"Done in {int((time.time() - start_time) * 1000)}ms.")

    def play_audio(self, file):
        print("Playing audio...")
        playsound(self, file)

    def get_goal_location(self):
        return self.goal_location
    
    def robot_introduction(self):
        # generating a response with openai
        generated_response = self.generate_response(
                "introduce yourself in one sentence then finish with asking where they want to go", 
                self.guidebot_personality
            )

        # converting response to speech that is played
        response_audio = self.text_to_speech(generated_response)

    
    def chatty_when_driving(self, location):
        #calibrate audio input
        background_noise_dbfs = self.measure_background_noise()
        silence_threshold = background_noise_dbfs - 2 # 3 worked decently; 2 has a higher chance of picking up random things
        print(f"Silence threshold: {silence_threshold}")

        personality = f'''
                you are a guide robot, you can guide people around the room '120', 
                you are currently guiding someone to {location} in the room. 
                Talk about your day and your thoughts on work breifly, talk in shorter sentences, and a maximim of 3 sentences.
            ''' 
        # generating a response with openai
        generated_response = self.generate_text(personality) #does having '' break it?

        # converting response to speech that is played
        response_audio = self.text_to_speech(generated_response)

    def generate_text(self, personality):
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": personality},
                {"role": "assistant", "content": 
                 '''Marlo and linden are robots who work at Emily's desk in room 120 aswell.
                 They work on a research project for emily although you aren't quite sure what it is.'''}

            ],
        )
        response_data = response.choices[0].message.content
        print("AI Response: ", response_data)

        return response_data.replace("'", "")
       
    def success_script(self):
        generated_response = self.generate_response(f"you've successfully reached the goal position {self.goal_location}", self.guidebot_personality)
        # converting response to speech that is played
        response_audio = self.text_to_speech(generated_response)

    def fail_script(self):
        generated_response = self.generate_response(f"you've have failed to reach the goal position, {self.goal_location}", self.guidebot_personality)
        # converting response to speech that is played
        response_audio = self.text_to_speech(generated_response)

    def cancel_script(self):
        generated_response = self.generate_response(f"the goal to {self.goal_location} has been canceled", self.guidebot_personality)
        # converting response to speech that is played
        response_audio = self.text_to_speech(generated_response)

    def reset(self):
        self.goal_location = 'unknown'
        # generating a response with openai
        generated_response = self.generate_response("ask the user for where else they would like to be guided to", self.guidebot_personality)

        # converting response to speech that is played
        response_audio = self.text_to_speech(generated_response)
        main(continued=True)


    def main(self, continued = False):
        
        if continued == False:
            # calibrating the audio input
            # background_noise_rms = measure_background_noise()
            # silence_threshold = background_noise_rms - 10 
            background_noise_dbfs = self.measure_background_noise()
            silence_threshold = background_noise_dbfs - 2 # 3 worked decently; 2 has a higher chance of picking up random things
            print(f"Silence threshold: {silence_threshold}")

            self.robot_introduction()
            self.goal_location = 'unknown'

        while self.goal_location.lower() == 'unknown': # changed from true to try to make it end when starting navigation
            # recording audio from microphone until a pause is detected
            audio_file = self.record_audio(threshold=silence_threshold)

            # converting speech to text
            query = self.speech_to_text(audio_file)

            # generating a response with openai
            generated_response = self.generate_response(query, self.guidebot_personality)

            # converting response to speech that is played
            response_audio = self.text_to_speech(generated_response)

            instructions = '''
                Where does the user want to be guided to, respond with the name of the location only, 
                if unclear respond with unknown
            ''' 
            generated_response = self.generate_response(query, self.guidebot_personality + instructions)
            self.goal_location = generated_response
            # playing the audio of response
            #play_audio(response_audio)

            # clear temporary files 
            os.remove(audio_file)
            # os.remove(response_audio)

def main(args=None):
    rclpy.init(args=args)

    converse_with_robot = OpenAIChatbot()
    converse_with_robot.main()

    # rclpy.spin(converse_with_robot)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
