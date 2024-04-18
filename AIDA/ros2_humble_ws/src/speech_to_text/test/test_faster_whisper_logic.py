from speech_to_text.faster_whisper_logic import FasterWhisperLogic
import librosa
from pathlib import Path
import numpy as np
import os
import pytest

# # It seems like some of the tests take to long and therefore are skipped
# # and gives us missing result file for the test and that is why we get a error
# # when running the tests

# Apperently there is a segmentation fault when running this test when
# running the tests for other files with both colcon test and python3 -m pytest
 
# def test_transcribe_audio():
#     """
#     Tests the transcribe_audio method in the FasterWhisperLogic class.
#     Tests with a audio file faster_whisper was trained on.
#     """
#     # Create a FasterWhisperLogic object
#     fwl = FasterWhisperLogic()

#     current_path = Path(os.path.dirname(os.path.realpath(__file__)))
#     audio_path = current_path / "test_resource" / "jfk.wav"

#     audio_data, _ = librosa.load(audio_path, sr=16000)
    
#     # Call the transcribe_audio method
#     segments = fwl.transcribe_audio(audio_data)
    
#     assert len(segments) == 1
#     #Remove leadning and trailing whitespaces    
#     result = segments[0].text.strip()
#     assert result ==  "And so my fellow Americans, ask not what your country can do for you, ask what you can do for your country."

def test_transcribe_audio_audio_out_of_range_greater():
    """
    Tests the transcribe_audio method in the FasterWhisperLogic class.
    Tests that the method raises a ValueError when the audio data is out of range.
    """
    # Create a FasterWhisperLogic object
    fwl = FasterWhisperLogic()

    audio_data = np.ndarray(shape=(4,), dtype=np.float32)
    audio_data[0] = 0.32
    audio_data[1] = -0.63
    audio_data[2] = 0.0
    audio_data[3] = 1.1
    
    # Call the transcribe_audio method
    with pytest.raises(ValueError) as exec_info:
        fwl.transcribe_audio(audio_data)
    assert exec_info.type == ValueError

def test_transcribe_audio_audio_out_of_range_smaller():
    """
    Tests the transcribe_audio method in the FasterWhisperLogic class.
    Tests that the method raises a ValueError when the audio data is out of range.
    """
    # Create a FasterWhisperLogic object
    fwl = FasterWhisperLogic()

    audio_data = np.ndarray(shape=(4,), dtype=np.float32)
    audio_data[0] = 0.32
    audio_data[1] = -0.63
    audio_data[2] = 0.0
    audio_data[3] = -1.1

    # Call the transcribe_audio method
    with pytest.raises(ValueError) as exec_info:
        fwl.transcribe_audio(audio_data)
    assert exec_info.type == ValueError

