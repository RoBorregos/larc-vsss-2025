import cv2
import numpy as np
import albumentations as A
import os
import random
# --- Configuration ---

INPUT_IMAGE_PATH = 'path/to/your/image.jpg'  # Replace with your image path
OUTPUT_DIR = 'augmented_images_only' # Directory to save augmented images
NUM_AUGMENTATIONS = 20 # How many augmented images to generate
ROTATION_LIMIT = 180 # Maximum rotation angle in degrees (e.g., 180 means -180 to +180 degrees)
SATURATION_LIMIT = 1 # Maximum saturation change factor (e.g., 1 means -100% to +100%)
BRIGHTNESS_CONTRAST_LIMIT = 1 # Maximum brightness/contrast change factor

# --- Main Augmentation Logic ---

# Create output directory if it doesn't exist
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Load the original image
image = cv2.imread(INPUT_IMAGE_PATH)
if image is None:
    print(f"Error: Could not load image from {INPUT_IMAGE_PATH}")
    exit()

# Convert image to RGB format as Albumentations works with RGB
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
img_height, img_width = image.shape[:2]

# Define the augmentation pipeline (without bounding box parameters)
# We use p=1.0 for transforms we always want to apply (like rotation/saturation variation)
# but the *degree* of transformation is randomized within the limits.
transform = A.Compose([
    # --- Rotation ---
    # Rotate the image by a random angle within [-ROTATION_LIMIT, +ROTATION_LIMIT] degrees.
    # border_mode=cv2.BORDER_CONSTANT ensures black borders for rotated areas.
    # value=[0,0,0] sets the border color to black.
    A.Rotate(limit=ROTATION_LIMIT, p=1.0, border_mode=cv2.BORDER_CONSTANT, value=[0,0,0]),

    # --- Saturation & Illumination ---
    # Randomly change hue, saturation, and value (brightness).
    # hue_shift_limit=0 means hue is not changed.
    # sat_shift_limit controls saturation change.
    # val_shift_limit controls brightness change.
    A.HueSaturationValue(hue_shift_limit=0, sat_shift_limit=SATURATION_LIMIT, val_shift_limit=BRIGHTNESS_CONTRAST_LIMIT, p=1.0),

    # --- Optional: Add other useful augmentations ---
    # A.RandomBrightnessContrast(brightness_limit=BRIGHTNESS_CONTRAST_LIMIT, contrast_limit=BRIGHTNESS_CONTRAST_LIMIT, p=0.5),
    # A.GaussNoise(var_limit=(10.0, 50.0), p=0.3),
    # A.GaussianBlur(blur_limit=(3, 7), p=0.3),
    # A.HorizontalFlip(p=0.5), # Flip horizontally with 50% probability
])

print(f"Generating {NUM_AUGMENTATIONS} augmented images...")

# Generate and save augmented images
for i in range(NUM_AUGMENTATIONS):
    try:
        # Apply the transformations to the image
        augmented = transform(image=image)
        augmented_image = augmented['image']

        # Generate unique output filenames
        base_filename = os.path.splitext(os.path.basename(INPUT_IMAGE_PATH))[0]
        output_image_path = os.path.join(OUTPUT_DIR, f"{base_filename}_aug_{i+1}.jpg")

        # Save the augmented image (convert back to BGR for OpenCV)
        cv2.imwrite(output_image_path, cv2.cvtColor(augmented_image, cv2.COLOR_RGB2BGR))

        print(f"Saved: {output_image_path}")

    except Exception as e:
        print(f"Error during augmentation iteration {i+1}: {e}")
        # Optionally continue to the next iteration or stop
        # continue

print("Augmentation complete.")
