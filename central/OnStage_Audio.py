### setup ###
# navigate to:
# /lib/systemd/system/bluetooth.service.d/nv-bluetooth-service.conf

# edit line:
# ExecStart=/usr/lib/bluetooth/bluetoothd -d --noplugin=audio,a2dp,avrcp
# to:
# ExecStart=/usr/lib/bluetooth/bluetoothd -d

# run:
# sudo apt-get update
# sudo apt-get install pulseaudio-module-bluetooth

### import libraries ###
import pygame

pygame.mixer.init()
pygame.mixer.set_num_channels(4)

channels = {
  "short_sfx": pygame.mixer.Channel(0),
  "bg_sfx": pygame.mixer.Channel(1),
  "commentary_sfx": pygame.mixer.Channel(2)
}

audios = {
  "mining": pygame.mixer.Sound("MINING.mp3")
}

# play audio for ice
def play_sound(channel, name):
  channels[channel].play(audios[name])

### testing ###
if __name__ == "__main__":
  play_sound("short_sfx", "mining")  # mining audio for ice collection
