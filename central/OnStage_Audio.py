### INFO ###
# navigate to:
# /lib/systemd/system/bluetooth.service.d/nv-bluetooth-service.conf

# edit line:
# ExecStart=/usr/lib/bluetooth/bluetoothd -d --noplugin=audio,a2dp,avrcp
# to:
# ExecStart=/usr/lib/bluetooth/bluetoothd -d

# run:
# sudo apt-get update
# sudo apt-get install pulseaudio-module-bluetooth

### import subfiles ###
import pygame

pygame.mixer.init()
pygame.mixer.set_num_channels(8)
pygame.mixer.music.set_volume(1.0)

channels = {
    "short_sfx": pygame.mixer.Channel(0),
    "bg_sfx": pygame.mixer.Channel(1),
    "commentary_sfx": pygame.mixer.Channel(2)
}

audios = {
    "bg": pygame.mixer.Sound("/home/vedaang/onstage/BG_MUSIC.mp3"),
    "mining": pygame.mixer.Sound("/home/vedaang/onstage/MINING.wav"),
    "watering": pygame.mixer.Sound("/home/vedaang/onstage/WATERING.mp3")
}

def play_sound(channel, name):
    if (channels[channel].get_busy()):
        pygame.mixer.find_channel().play(audios[name], maxtime=5000)
    else:
        channels[channel].play(audios[name], maxtime=5000)
  
def play_bg():
    channels["bg_sfx"].play(audios["bg"], loops=-1)
    
def play_mining():
    play_sound("short_sfx", "mining")
    
def play_watering():
    play_sound("short_sfx", "watering")

### testing ###
if __name__ == "__main__":
    play_bg()
