import whisper
import numpy as np

class SpeechToText():
    def __init__(self, model: str = None) -> None:
        """
        Initializes the SpeechToText object for speech to text conversion.
        
        Args:
            model : str : The model to use for speech to text conversion.
            
        Returns:
            None
            
        """
        model = model or "base.en"
        self.model = whisper.load_model(model)
    
    def transcribe(self, audio_data: np.ndarray) -> list:
        """
        Translates the received audio data to text.
        
        Args:
            audio_data: A 1D numpy array representing the received audio data.
        
        Returns:
            The transcribed text.
        """
        # Check if the audio data is in the correct range for transcription
        if np.any(audio_data < -1) or np.any(audio_data > 1):
            raise ValueError("Audio data is not in the correct range for transcription")

        result = self.model.transcribe(audio_data)

        return result.get("text", "")

