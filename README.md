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
* PNG library (1.2.54-1ubuntu1.1)
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

## 3. Data Format

The input is a rosbag file and the topic where the images were published. The output is the sequence of images, compressed in PNG format and a text file with correspoinding timestamps: for image 0000.png the associated timestamp is stored in row 0, for 01000.png in row 1000 and so on.

## 4. Usage

The `bag_to_image` tool can invoked as follows:
```
bag_to_image [options] <rosbag> <topic>
```
where the available options are:

* `--images_path=desired_path`: to set the output images path

* `--timestamps_file=desired_file`: to set the output timestamps file

* `--show_images`: to preview the images during extraction
