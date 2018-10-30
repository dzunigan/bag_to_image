# bag_to_image
Extract images and timestamps from a rosbag file.

**Author:** [David Zuñiga-Noël](http://mapir.isa.uma.es/mapirwebsite/index.php/people/270)

**License:**  [GPLv3](https://raw.githubusercontent.com/dzunigan/calibration2d/master/LICENSE.txt)

## 1. Dependencies

* CMake (3.5.1-1ubuntu1)
   ```
   sudo apt install cmake
   ```
* ROS (Kinetic Kame)

   See [ROS installation](http://wiki.ros.org/kinetic/Installation).

* Gflags (2.1.2-3)
   ```
   sudo apt install libgflags-dev
   ```
* libpng (1.2.54-1ubuntu1.1)
   ```
   sudo apt install libpng-dev
   ```
   
## 2. Build

Once all dependencies are installed, proceed to build the source code with the automated build script provided. Simply run the following commands:
```
git clone https://github.com/dzunigan/bag_to_image.git
cd bag_to_image
bash build.sh
```

The compiled executable should be inside the `bin` directory.

## 3. Data Format

The input is a rosbag file and the topic where the images were published. The output is the sequence of images, compressed in PNG format and a CSV file with correspoinding timestamps in nanoseconds.

## 4. Usage

The `bag_to_image` tool can invoked as follows:
```
bag_to_image [options] <rosbag> <topic>
```
where the available options are:

* `--output_path=desired_path`: to set the output root directory

* `--images_dir=desired_dir`: to set the images directory name (inside output directory)

* `--timestamps_file=desired_file`: to set the output timestamps filename (inside output directory)

* `--compression_level=compression_level`: to set the PNG compression level (between 0-9)

* `--show_images`: to preview the images during extraction
