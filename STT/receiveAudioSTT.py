import numpy as np
from faster_whisper import WhisperModel
import decodeAudio


class AudioReceiverSTT:
    """
    This class represents an audio receiver that subscribes to audio data and translates it to text.

    Attributes:
        model: A WhisperModel object representing the model for translating audio data to text.

    Methods:
        __init__: Initializes the AudioReceiverSTT object.
        translate: Translates the received audio data to text.
    """

    def __init__(self, model_size: str=None):
        """
        Initializes the AudioReceiverSTT object.

        Args:
            model_size: A string representing model to be used for translating audio data to text.

        Returns:
            None
        """
        if model_size is not None:
            self.model = WhisperModel(model_size, device="auto")
        else:
            default_model = "base.en"
            self.model = WhisperModel(default_model)

    def translate(self, audio_data):
        """
        Translates the received audio data to text.

        Args:
            audio_data: A 2D numpy array representing the received audio data.

        Returns:
            A list of semgents representing the translated text.
        """

        segments, _ = self.model.transcribe(audio_data, beam_size=5)
        return list(segments)
    


   

def fileToAudioData(file_path : str) -> np.ndarray:
    """
    Converts a file to numpy array with audio data.

    Args:
        file_path: A string representing the path to the file.
    
    Returns:
        A numpy array representing the audio data.
    """
    return decodeAudio.decode_audio(file_path)

if __name__ == "__main__":
    audio_data = fileToAudioData("/home/alexander/Projects/Kandidat/PUM-04/STT/resources/Move3mForwardTurnRight2.wav")
    audio_receiver = AudioReceiverSTT("tiny.en")
    segments = audio_receiver.translate(audio_data)
    for segment in segments:
        print(segment.text)



"We could also add vad_filter to filter out silence from the audio data."
"In segments there seems to be a list of tokens, which we can use to increase the accuracy of the translation as we have specific commands and words that we are looking for."