import serial
import time
from ubidots import ApiClient

print("Program Started")

api = ApiClient(token ='A1E-29vxkjMuJNeDWl8TMnrWwPXaI5VCT3') #update token

my_lat = api.get_variable('5bf1f3f4c03f9753db93bf7b') #
my_lon = api.get_variable('5bf1f404c03f9753db93bf7e')
                          
their_lon = api.get_variable('5bf1f40fc03f9753db93bf83')
their_lat = api.get_variable('5bf1f41ac03f9753bce9ef56')
                             
ser = serial.Serial('/dev/ttyACM0', 9600) #update with port for Arduino

count = 2 # controls the while loop

while True:
        while (count == 2): #Read and upload this Devices GPS Lat
                read_serial = ser.read(13)
                print("This Devices Latitude has been accepted")
                print(read_serial)
                input_lat = (float(read_serial))
                print(input_lat)
                new_value = my_lat.save_value({'value': input_lat})
                count = 1
        while (count == 1): #Read and upload this Devices GPS Lon
                read_serial = ser.read(13)
                print("This Devices Longitude has been accepted")
                print(read_serial)
                input_lon = (float(read_serial))
                print(input_lon)
                new_value = my_lon.save_value({'value': input_lon})
                count = 0
        while (count == 0):
                time.sleep(2)
                last_lat = my_lat.get_values(1) #CHANGE TO THEIR LAT ONCE SECOND DEVICE EXISTS
                ser_write = str(last_lat[0]['value'])
                print (ser_write)
                ser.write(bytes(ser_write, 'UTF-8')) #placeholder foreign GPS Lat
                time.sleep(2)
                last_lon = my_lon.get_values(1) #CHANGE TO THEIR LON ONCE SECOND DEVICE EXISTS
                ser_write = str(last_lon[0]['value'])
                print (ser_write)
                ser.write(bytes(ser_write, 'UTF-8')) #Placeholder foreign GPS Long
                count = 2
                time.sleep(2)

        
