import cv2
import numpy as np
from pathlib import Path


output_dir = Path(
    "/home/sea/scrobot_ws/src/"
    "scrobot_simulation/models/"
    "court_apriltags/materials/textures"
)

output_dir.mkdir(
    parents=True,
    exist_ok=True
)

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_16h5
)

# tag16h5:
#
# 4x4 information cells
# + 1 black border cell each side
# = 6x6 border-to-border
#
# Add another white cell around the outside:
# total = 8x8 equivalent.
#
# 384 px / 6 cells = 64 px/cell
# quiet border = 64 px
# final image = 512 x 512

marker_pixels = 384
quiet_border = 64


for tag_id in range(4):

    try:
        marker = cv2.aruco.generateImageMarker(
            dictionary,
            tag_id,
            marker_pixels,
            borderBits=1
        )

    except AttributeError:
        # Older OpenCV API
        marker = np.zeros(
            (marker_pixels, marker_pixels),
            dtype=np.uint8
        )

        cv2.aruco.drawMarker(
            dictionary,
            tag_id,
            marker_pixels,
            marker,
            1
        )

    texture = cv2.copyMakeBorder(
        marker,
        quiet_border,
        quiet_border,
        quiet_border,
        quiet_border,
        cv2.BORDER_CONSTANT,
        value=255
    )

    filename = output_dir / f"tag{tag_id}.png"

    cv2.imwrite(
        str(filename),
        texture
    )

    print(
        f"generated {filename}"
    )
