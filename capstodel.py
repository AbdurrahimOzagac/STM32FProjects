import keyboard
import time

print("Harika! Program çalışıyor...")
print("Artık Caps Lock tuşuna bastığında Backspace gibi silecek.")

# Caps Lock tuşunu Backspace'e yönlendiriyoruz
keyboard.remap_key('caps lock', 'backspace')

# Programın kapanmaması için sonsuza kadar çalışmasını sağlıyoruz
while True:
    time.sleep(1)