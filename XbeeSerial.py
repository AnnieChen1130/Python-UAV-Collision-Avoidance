import serial
import time

#setting up xbee communication
ser = serial.Serial(
    
    port='COM9',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1   
)

# allows reading the serial fully and avoid missing any bytes. *** MUST USE \n AT THE END OF LINE ***
def serialread():
    t_end = time.time() + 4             #set timer of 4 seconds
    time.sleep(.001)                    # delay of 1ms
    val = ser.readline()                # read complete line from serial output
    while not '\\n'in str(val):         # check if full data is received. 
        # This loop is entered only if serial read value doesn't contain \n
        # which indicates end of a sentence. 
        # str(val) - val is byte where string operation to check `\\n` 
        # can't be performed
        time.sleep(.001)                # delay of 1ms 
        temp = ser.readline()           # check for serial output.
        if not not temp.decode():       # if temp is not empty.
            val = (val.decode()+temp.decode()).encode()
            # requrired to decode, sum, then encode because
            # long values might require multiple passes
        elif time.time() < t_end:       # break loop if alloted time for serial reading is passed.
            break
    val = val.decode()                  # decoding from bytes
    val = val.strip()                   # stripping leading and trailing spaces.
    return val


while True:
    
    msg = ser.readline().decode()
    print(msg)
 
    '''
    msg = serialread()
    
    if msg:
     
		parsed_string = msg.split(':')
		
		if parsed_string[0] == "ICAO": 
			print ('ICAO: ', parsed_string[1])
		elif parsed_string[0] == "Lattitude": 
			print ('Lattitude: ', parsed_string[1])
		elif parsed_string[0] == "Longitude": 
			print ('Longitude: ', parsed_string[1])    
		elif parsed_string[0] == "Altitude": 
			print ('Altitude: ', parsed_string[1])    
		elif parsed_string[0] == "Velocity": 
			print ('Velocity: ', parsed_string[1])   
		elif parsed_string[0] == "Airspeed": 
			print ('Airspeed: ', parsed_string[1])  
    '''