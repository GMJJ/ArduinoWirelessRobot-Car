import serial
import time
import asyncio
import websockets
import atexit
import threading

#port = '/dev/ttyACM0'
port = "Com13"
ser = serial.Serial(port, 115200, timeout=0.05)
ser.reset_input_buffer()

leftSpeed = 0
rightSpeed = 0

def sendSerial(left, right):
    message = f"{left},{right}\n"
    ser.write(message.encode('utf8'))
    line = ser.readline().decode('utf-8').rstrip()
    return line
    
def updateSerial():
    while(True):
        sendSerial(leftSpeed, rightSpeed)

serialThread = threading.Thread(target=updateSerial)
serialThread.start()

async def handleSocket(ws):
    global leftSpeed
    global rightSpeed
    try:
        while(True):
            recievedData = await ws.recv()
            print(recievedData)
            leftSpeed = float(recievedData.split(",")[0])+float(recievedData.split(",")[1])
            rightSpeed = float(recievedData.split(",")[0])-float(recievedData.split(",")[1])
            if(abs(leftSpeed)<.2):
                leftSpeed = 0;
            if(abs(rightSpeed)<.2):
                rightSpeed = 0;
            if(leftSpeed>1):
                leftSpeed = 1
            if(leftSpeed<-1):
                leftSpeed = -1
            if(rightSpeed>1):
                rightSpeed = 1
            if(rightSpeed<-1):
                rightSpeed = -1
            await ws.send("Roger")
    except:
        pass



async def main():
    async with websockets.serve(handleSocket, None, 3000):
        await asyncio.Future()  # run forever       

if __name__ == '__main__':
    asyncio.run(main())
atexit.register(sendSerial, (0,0))