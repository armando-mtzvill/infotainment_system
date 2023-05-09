import serial
import time
import os
import vlc

dir = '/home/pepe/Music/'

ser = serial.Serial ("/dev/ttyS0", 9600, timeout=1000,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE)    #Open port with baud rate


instance = vlc.Instance('--aout=alsa')

with os.scandir(dir) as ficheros:
    ficheros = [fichero.name for fichero in ficheros if fichero.is_file()]
    
player = vlc.Instance()

media_player = vlc.MediaListPlayer()

media_list = vlc.MediaList()

for i in range (len(ficheros)):
    media = player.media_new(dir + ficheros[i])
    media_list.add_media(media)

media_player.set_media_list(media_list)

index=-1
t = 0.2
vol=100
dataRed = "0"
while True:
                 #transmit data serially
    dataRed = ser.read().decode("Ascii") 
    if (dataRed == "2") :
        media_player.play()
        ser.write("}".encode("Ascii"))
        time.sleep(t)
        if (index == -1):
            index =0
        dataT = ficheros[index]
        size= len(dataT)
        for i in range (size-3) :
            ser.write(dataT[i].encode("Ascii"))
            time.sleep(t)
        #ser.write(sizekey)
    elif (dataRed == "8"):
        media_player.stop()
        ser.write("}".encode("Ascii"))
        time.sleep(t)
        index = -1
        dataT = "Stopped."
        size= len(dataT)
        for i in range (size) :
            ser.write(dataT[i].encode("Ascii"))
            time.sleep(t)
    
    elif (dataRed == "5"):
        media_player.pause()
        
    elif (dataRed == "6"):
        ser.write("}".encode("Ascii"))
        time.sleep(t)
        
        if (index < len(ficheros)-1):
            media_player.next()
            index += 1
        else:
            media_player.stop()
            media_player.play()
            index = 0
        dataT = ficheros[index]
        size= len(dataT)
        for i in range (size-3) :
            ser.write(dataT[i].encode("Ascii"))
            time.sleep(t)
        
    elif (dataRed == "4"):
        media_player.previous()
        if (index < 0 ):
            index = 0
            ser.write("}".encode("Ascii"))
            time.sleep(t)
            dataT = ficheros[index]
            size = len(dataT)
            for i in range (size-3) :
                ser.write(dataT[i].encode("Ascii"))
                time.sleep(t)
            
        elif (index > 0):
            ser.write("}".encode("Ascii"))
            time.sleep(t)
            index -= 1
            dataT = ficheros[index]
            size = len(dataT)
            for i in range (size-3) :
                ser.write(dataT[i].encode("Ascii"))
                time.sleep(t)
        elif (index == 0):
            ser.write(".".encode("Ascii"))
            media_player.stop()
            media_player.play()
        
    elif (dataRed == "A"):
        player = media_player.get_media_player()
        if (vol <= 90):
            vol = vol + 10 #Aumentar el volumen hasta llegar a 100
        else :
            vol = 100
        player.audio_set_volume(vol) #Cambiar el volumen al nuevo valor    
    
    elif (dataRed == "B"):
        player = media_player.get_media_player() 
        if (vol >= 10):
            vol = vol - 10 #Reducir el volumen hasta llegar a 0
        else :
            vol = 0
        player.audio_set_volume(vol) # Cambiar el volumen al nuevo valor
    
     elif (dataRed == "C"):
        player = media_player.get_media_player() 
        mute = player.audio_get_mute()
        player.audio_set_mute(not mute) # Cambiar el estado del mute entre true y false
        
    time.sleep(.4)




