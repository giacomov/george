import time

import pkg_resources

import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from jetbot.utils.utils import get_ip_address


class Display:

    def __init__(self):

        # setting gpio to 1 is hack to avoid platform detection
        self._disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1) 

        # Try to connect to the display 3 times
        for _ in range(3):
            
            try:
                # Try to connect to the OLED display module via I2C.
                self._disp.begin()
            except OSError as err:
                print("OS error: {0}".format(err))
                time.sleep(10)
            else:
                break
    
    def _get_resource_path(self, resource_name):
        return pkg_resources.resource_filename('navigation', f'resources/{resource_name}')

    def welcome(self):
        # Clear display.
        self._disp.clear()
        self._disp.display()
        
        # Create blank image for drawing.
        width = self._disp.width
        height = self._disp.height

        # Figure out path of jetbot_2.png 
        image_path = self._get_resource_path('jetbot_2.png')

        # Create a little animation to welcome the user
        # The image is displayed in 5 steps, each step is displayed for 0.5 seconds
        # In the first step the image is 10% of its size, in the last step it is 100% of its size
        for scale in [0.1, 0.35, 0.6, 0.85, 1.0]:
            # Resize the image
            image = (
                Image
                .open(image_path)
                .resize(
                    (
                        int(width * scale), 
                        int(height * scale)
                    ),
                    Image.ANTIALIAS)
                .convert('1')
            )
            
            # Pad the image to be 128x32, with the original image in the center
            image = Image.new('1', (width, height))
            image.paste(image, (int((width - image.width) / 2), int((height - image.height) / 2)))

            self._disp.image(image)
            self._disp.display()
            time.sleep(0.5)
        

