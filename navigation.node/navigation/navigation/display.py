import time

import pkg_resources

import Adafruit_SSD1306
import math

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
from PIL import ImageOps
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
    
    def _clear(self):
        self._disp.clear()
        self._disp.display()

    def welcome(self):
        # Clear display.
        self._clear()
        
        # Create blank image for drawing.
        width = self._disp.width
        height = self._disp.height

        # Figure out path of jetbot_2.png 
        image_path = self._get_resource_path('jetbot_2.png')

        # Create a little animation to welcome the user
        # The image is displayed in 5 steps, each step is displayed for 0.5 seconds
        # In the first step the image is 10% of its size, in the last step it is 100% of its size
        for scale in [0.1, 0.35, 0.6, 0.85, 1.0]:

            # Open image
            image = Image.open(image_path)

            # Invert colors
            image = ImageOps.invert(image)

            # Resize the image
            resized_image = (
                image
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
            image.paste(
                resized_image, 
                (
                    int((width - resized_image.width) / 2), 
                    int((height - resized_image.height) / 2)
                )
            )

            self._disp.image(image)
            self._disp.display()
            time.sleep(0.5)
        
        self._banner(width, height)
    
    def _banner(self, width, height):

        self._clear()

        image = Image.new('1', (width, height))

        # Load default font.
        font = ImageFont.load_default()

        # Create drawing object.
        draw = ImageDraw.Draw(image)

        # Define text and get total width.
        text = 'HELLO THERE! I AM GEORGE, A VOICE-ACTIVATED LITTLE BOT. SPEAK TO ME AND TELL ME WHAT TO DO!'
        maxwidth, unused = draw.textsize(text, font=font)

        # Set animation and sine wave parameters.
        amplitude = height/4
        offset = height/2 - 4
        velocity = -3
        startpos = width

        # Animate text moving in sine wave.
        pos = startpos

        while True:
            # Clear image buffer by drawing a black filled box.
            draw.rectangle((0,0,width,height), outline=0, fill=0)
            # Enumerate characters and draw them offset vertically based on a sine wave.
            x = pos
            for _, c in enumerate(text):
                # Stop drawing if off the right side of screen.
                if x > width:
                    break
                # Calculate width but skip drawing if off the left side of screen.
                if x < -10:
                    char_width, _ = draw.textsize(c, font=font)
                    x += char_width
                    continue

                # Calculate offset from sine wave.
                y = offset+math.floor(amplitude*math.sin(x/float(width)*2.0*math.pi))
                # Draw text.
                draw.text((x, y), c, font=font, fill=255)
                # Increment x position based on chacacter width.
                char_width, _ = draw.textsize(c, font=font)
                x += char_width
            # Draw the image buffer.
            self._disp.image(image)
            self._disp.display()
            # Move position for next frame.
            pos += velocity
            # Return if off the left side of screen.
            if pos < -maxwidth:
                pos = startpos
                
                self._clear()
                return