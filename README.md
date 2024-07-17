## Video Publisher Method

This guide provides step-by-step instructions for setting up and running the Video Publisher project in a ROS Node. Follow these steps to ensure proper setup and execution.

### Setup and Execution

1. **Clean the Build Environment**
   - Navigate to the main project directory and remove the `build` and `devel` directories if they exist:
     ```
     rm -r build devel
     ```

2. **Configure Video Path**
   - Update the video path in the `video_playback.launch` file located at `/src/image_publisher/launch` with the path to your `.h264` video file.

3. **Build the Project**
   - In the main folder, build the project using the `catkin_make` command:
     ```
     catkin_make
     ```

4. **Source Setup Script**
   - Source the environment setup script to update your environment:
     ```
     source devel/setup.bash
     ```

5. **Launch Video Playback**
   - Change directory to where `video_playback.launch` is located and launch the video playback:
     ```
     cd src/image_publisher/launch
     roslaunch video_playback.launch
     ```

6. **Verify Video Publishing**
   - Check if the video is being published successfully:
     ```
     rostopic list
     ```

7. **Restart ROS Core**
   - If needed, terminate the `roslaunch`, start `roscore` first, and then repeat the video playback launch:
     ```
     roslaunch video_playback.launch
     ```

8. **Camera Calibration**
   - Run the camera calibration tool:
     ```
     rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/cam0/image_raw
     ```
