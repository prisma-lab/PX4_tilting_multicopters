# Motor control message
uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

uint8 NUM_ACTUATORS = 8

float32[16] value

# TOPICS null_space_vector