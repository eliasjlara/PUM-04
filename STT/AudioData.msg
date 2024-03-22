# AudioData.msg

# Header for the message
std_msgs/Header header

# The audio data, as a byte array
uint8[] data


# The sample rate of the audio data
int32 sample_rate

# The number of channels in the audio data
int32 channels

# The audio sample count
int32 samples