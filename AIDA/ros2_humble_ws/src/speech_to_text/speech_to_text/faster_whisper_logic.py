from faster_whisper import WhisperModel
import numpy as np

class FasterWhisperLogic():
    def __init__(self, model : str=None) -> None:
        """
        Initializes the FasterWhisperLogic object for speech to text conversiton.
        
        Args:
            model : str : The model to use for speech to text conversion.
            
        Returns:
            None
            
        """
        # If no model is provided, use the base.en model
        if model is None:
            model = "base.en"

        self.model = WhisperModel(model)
    
    def transcribe_audio(self, audio_data : np.ndarray) -> list:
        """
        Translates the received audio data to text.
        
        Args:
            audio_data: A 1D numpy array representing the received audio data.
        
        Returns:
            A list of segments representing the translated text.
        """
        # Check the audio data if it is in the correct range for transcribtion
        if np.any(audio_data<-1) or np.any(audio_data>1):
            raise ValueError("Audio data is not in the correct range for transcribtion")
        

        #TODO : maybe add beam_size as parameter as well as which language we are using
        segments, _ = self.model.transcribe(audio_data, beam_size=5)
        # Segments is iterable, for trancription of data we need to iterate over it
        # Can be done by placing the segments in a list
        return list(segments)
    
    #TODO : Maybe place helpers for converting the message to numpy array in a separate file in audio_data folder
    def message_to_numpy_array(self, msg) -> np.ndarray:
        """ 
        Converts an ros message from topic to an np array for use
        to apply STT
        
        Args:
            msg: An AudioData object representing the received audio data
        
        Returns: 
            Float32 numpy array in the range [-1, 1]
        """
        if msg.samples == 0:
            raise ValueError("No audio data in message")
        #TODO : Add error handling
        return np.frombuffer(msg.data, dtype=np.float32)
    
    #TODO : Maybe add method which uses the defined commands to improve accuracy of transcription
