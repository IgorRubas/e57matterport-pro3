# e57matterport-pro3
Python example: fast slicing, denoising and DXF generation
The included Python workflow allows you to define slice step and cut thickness independently for each axis (X/Y/Z), apply noise removal, and generate clean 2D sections from large E57 files.

On a typical 5–6 GB E57 scan from Matterport Pro3, the script produces DXF cross‑sections of a 46 × 78 × 12 m hall in about 15–20 minutes, depending on hardware and slice density.

This makes it practical to extract usable 2D geometry from noisy Pro3 data, even though the sensor’s accuracy limits prevent reliable automated feature detection.

How to Run
Place your E57 file in the project folder and name it model.e57.

Install dependencies:

Kod
pip install numpy pye57 ezdxf open3d
Run the script:

Kod
python main.py
The script loads the E57, removes noise, applies slicing with custom step and thickness per axis, and exports DXF sections.

Results are saved in the output_slices_dxf/ folder.
