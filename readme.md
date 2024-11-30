Instructions: 


Building the package:

    -> Clone the repo into src/ of your workspace.
    
      ''' cd ~/<your_ros2_ws>/src
       git clone https://github.com/Sherlock-21/Video_Conv.git'''
       
    -> Build the package
    
       cd ~/<your_ros2_ws> 
       colcon build
       
    -> Source the workspace
    
       source ~/<your_ros2_ws>/install/setup.bash

       
How to use the launch file:

    -> Run the launch file in the video_conv package
    
       ros2 launch video_conv video_conv_launch.py
       
    -> input_topic and output_topic can be modified by the user:
    
       ros2 launch video_conv video_conv_launch.py input_topic:=/camera/image_raw output_topic:=/converted_image
    
       (default input_topic:=/camera/image_raw output_topic:=/converted_image)
       
How to call the service to change the mode:

    -> Open a new terminal
    
    Switch to Grayscale Mode:
       ros2 service call /toggle_mode std_srvs/srv/SetBool "{data: true}"
       
    Switch to Colour Mode:
       ros2 service call /toggle_mode std_srvs/srv/SetBool "{data: false}"
       
       
How to visualize the output topic:

    ros2 run rqt_image_view rqt_image_view

    -> select /converted_image in the drop down to view the output topic.
    
    
    
      
        
