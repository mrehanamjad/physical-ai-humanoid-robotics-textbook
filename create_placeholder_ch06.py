import os
from PIL import Image, ImageDraw, ImageFont

def create_placeholder_image(text, file_path, width=800, height=400):
    """Creates a placeholder image with the given text."""
    if not os.path.exists(os.path.dirname(file_path)):
        os.makedirs(os.path.dirname(file_path))

    img = Image.new('RGB', (width, height), color = (73, 109, 137))
    d = ImageDraw.Draw(img)
    
    try:
        font = ImageFont.truetype("arial.ttf", 30)
    except IOError:
        font = ImageFont.load_default()

    lines = text.split('\n')
    
    total_text_height = 0
    line_heights = []
    for line in lines:
        bbox = d.textbbox((0, 0), line, font=font)
        line_height = bbox[3] - bbox[1]
        line_heights.append(line_height)
        total_text_height += line_height

    y_text = (height - total_text_height) / 2

    for i, line in enumerate(lines):
        bbox = d.textbbox((0, 0), line, font=font)
        text_width = bbox[2] - bbox[0]
        x = (width - text_width) / 2
        d.text((x, y_text), line, fill=(255, 255, 255), font=font)
        y_text += line_heights[i]
        
    img.save(file_path)
    print(f"Created placeholder image: {file_path}")

if __name__ == "__main__":
    create_placeholder_image("ROS 2 Node Lifecycle\nInit -> Create Node -> Spin -> Shutdown", "textbook/static/img/ch06-node-lifecycle.png")
    create_placeholder_image("Callback Data Flow\nSpin Loop -> Event Queue -> Execute Callback", "textbook/static/img/ch06-callback-flow.png")
